#include "ros/ros.h"
#include <Eigen/Dense>
#include <string>
#include <random>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <geometry_msgs/TransformStamped.h>
#include <uwb_driver/UwbRange.h>

#include <tf/transform_broadcaster.h>

using namespace std;
using namespace Eigen;

// Listing the IDs of the UWB nodes
std::vector<int> nodes_id;
// // Indices of the UWB nodes as ordered in the param passed to nodes_id
// std::vector<double> nodes_idx;
// Listing the position of the UWB nodes
std::vector<double> nodes_pos;

int nodes_total = 0;

// Listing the position of the antenna node
std::vector<std::vector<double>> antennas_pos;

// The transform between ground truth body and true body
map<int, Matrix<double, 3, 3> > R_V_B;
map<int, Matrix<double, 3, 1> > t_V_B;


// Slot map
std::vector<std::vector<int>> slotmap;
std::vector<double> slotmap_time;

// Ground truth topic
std::vector<std::string> ground_truth_topic;

std::vector<geometry_msgs::TransformStamped> nodes_info_msg;
std::vector<bool> nodes_info_new;

// Subscriber of ground truth topic
std::vector<ros::Subscriber> ground_truth_sub;

// Publisher of ground truth in rviz viewable format
std::vector<ros::Publisher> ground_truth_pub;
//Publish of the ground truth converted to the body frame
std::vector<ros::Publisher> ground_truth_body_pub;

// Publisher of the distance
std::vector<ros::Publisher> range_pub;

ros::Timer rn_timer;

double loss_chance;
vector<double> range_noise_gaussian;

int find_node_idx(int node_id)
{
    auto node_id_it = std::find(std::begin(nodes_id), std::end(nodes_id), node_id);
    if (node_id_it == std::end(nodes_id))
        return -1;
    else
        return int(std::distance(nodes_id.begin(), node_id_it));
}

void ground_truth_cb(const geometry_msgs::TransformStamped::ConstPtr& msg, int i)
{
    // printf("Received update for node %d\n", nodes_id[i]);
    nodes_info_msg[i] = *msg;
    nodes_info_new[i] = true;

    static std::vector<geometry_msgs::PoseStamped> ground_truth_viz_msg(nodes_id.size(),
                                                                        geometry_msgs::PoseStamped());
    
    ground_truth_viz_msg[i].header = nodes_info_msg[i].header;

    ground_truth_viz_msg[i].pose.position.x = nodes_info_msg[i].transform.translation.x;
    ground_truth_viz_msg[i].pose.position.y = nodes_info_msg[i].transform.translation.y;
    ground_truth_viz_msg[i].pose.position.z = nodes_info_msg[i].transform.translation.z;

    ground_truth_viz_msg[i].pose.orientation.x = nodes_info_msg[i].transform.rotation.x;
    ground_truth_viz_msg[i].pose.orientation.y = nodes_info_msg[i].transform.rotation.y;
    ground_truth_viz_msg[i].pose.orientation.z = nodes_info_msg[i].transform.rotation.z;
    ground_truth_viz_msg[i].pose.orientation.w = nodes_info_msg[i].transform.rotation.w;

    ground_truth_pub[i].publish(ground_truth_viz_msg[i]);


    // Calculate the body frame pose
    Vector3d p_W_V(nodes_info_msg[i].transform.translation.x,
                   nodes_info_msg[i].transform.translation.y,
                   nodes_info_msg[i].transform.translation.z);

    Quaterniond q_W_V(nodes_info_msg[i].transform.rotation.w,
                      nodes_info_msg[i].transform.rotation.x,
                      nodes_info_msg[i].transform.rotation.y,
                      nodes_info_msg[i].transform.rotation.z);

    int node_id = nodes_id[i];

    Quaterniond q_W_B     = q_W_V*Quaterniond(R_V_B[node_id]);
    Vector3d    t_V_B_inW = q_W_V.toRotationMatrix()*t_V_B[node_id];

    Vector3d p_W_B = p_W_V + t_V_B_inW;

    ground_truth_viz_msg[i].pose.position.x = p_W_B.x();
    ground_truth_viz_msg[i].pose.position.y = p_W_B.y();
    ground_truth_viz_msg[i].pose.position.z = p_W_B.z();

    ground_truth_viz_msg[i].pose.orientation.x = q_W_B.x();
    ground_truth_viz_msg[i].pose.orientation.y = q_W_B.y();
    ground_truth_viz_msg[i].pose.orientation.z = q_W_B.z();
    ground_truth_viz_msg[i].pose.orientation.w = q_W_B.w();

    ground_truth_body_pub[i].publish(ground_truth_viz_msg[i]);

    nodes_pos[i*3]     = nodes_info_msg[i].transform.translation.x;
    nodes_pos[i*3 + 1] = nodes_info_msg[i].transform.translation.y;
    nodes_pos[i*3 + 2] = nodes_info_msg[i].transform.translation.z;

    return;
}

void timer_cb(const ros::TimerEvent&)
{
    // Skip a few calls to avoid a quirk
    static int skips = 0;
    if(skips < 5)
    {
        skips++;
        return;
    }

    static int slot_idx = 0;
    static bool first_call = true;
    static ros::Time prev_time = ros::Time::now();

    if(first_call)
    {
        first_call = false;
        prev_time = prev_time - ros::Duration(slotmap_time[slot_idx]);
    }

    double true_time  = (ros::Time::now() - prev_time).toSec();
    double delay_time =  true_time - slotmap_time[slot_idx];

    printf("Slot: %d. Prev time: %f. Curr Time: %f. True time: %.3f. Intended time: %.3f. Delay: %f\n",
            slot_idx,
            prev_time.toSec(),
            ros::Time::now().toSec(),
            true_time,
            slotmap_time[slot_idx],
            delay_time);

    ros::Time proc_start = ros::Time::now();
    
    int transactions = slotmap[slot_idx].size()/4;
    if (transactions != 0)
    {
        //Calculate the distance and publish
        for(int j = 0; j < transactions; j++)
        {
            int rqst_id = slotmap[slot_idx][j*4];
            int rspd_id = slotmap[slot_idx][j*4 + 1];
            int rqst_idx = find_node_idx(rqst_id);
            int rspd_idx = find_node_idx(rspd_id);
            int ant_rqst_idx = slotmap[slot_idx][j*4 + 2];
            int ant_rspd_idx = slotmap[slot_idx][j*4 + 3];

            bool rqst_pos_updated = (nodes_pos[rqst_idx*3] != 9999) &&
                                    (nodes_pos[rqst_idx*3 + 1] != 9999) &&
                                    (nodes_pos[rqst_idx*3 + 2] != 9999);
            bool rspd_pos_updated = (nodes_pos[rspd_idx*3] != 9999) &&
                                    (nodes_pos[rspd_idx*3 + 1] != 9999) &&
                                    (nodes_pos[rspd_idx*3 + 2] != 9999);

            bool node_pos_updated = rqst_pos_updated && rspd_pos_updated && nodes_info_new[rqst_idx];

            if(node_pos_updated)
            {
                nodes_info_new[rqst_idx] = false;

                Vector3d rqst_pos(nodes_pos[rqst_idx*3],
                                  nodes_pos[rqst_idx*3 + 1],
                                  nodes_pos[rqst_idx*3 + 2]);
                Vector3d rqst_ant_pos(antennas_pos[rqst_idx][ant_rqst_idx*3],
                                      antennas_pos[rqst_idx][ant_rqst_idx*3 + 1],
                                      antennas_pos[rqst_idx][ant_rqst_idx*3 + 2]);
                Eigen::Quaterniond rqst_quat(nodes_info_msg[rqst_idx].transform.rotation.w,
                                             nodes_info_msg[rqst_idx].transform.rotation.x,
                                             nodes_info_msg[rqst_idx].transform.rotation.y,
                                             nodes_info_msg[rqst_idx].transform.rotation.z);

                Vector3d rspd_pos(nodes_pos[rspd_idx*3],
                                  nodes_pos[rspd_idx*3 + 1],
                                  nodes_pos[rspd_idx*3 + 2]);
                Vector3d rspd_ant_pos(antennas_pos[rspd_idx][ant_rspd_idx*3],
                                      antennas_pos[rspd_idx][ant_rspd_idx*3 + 1],
                                      antennas_pos[rspd_idx][ant_rspd_idx*3 + 2]);
                Eigen::Quaterniond rspd_quat(nodes_info_msg[rspd_idx].transform.rotation.w,
                                             nodes_info_msg[rspd_idx].transform.rotation.x,
                                             nodes_info_msg[rspd_idx].transform.rotation.y,
                                             nodes_info_msg[rspd_idx].transform.rotation.z);

                // Transform the rqst_pos and the rspd_pos from the vicon body to the true body
                Quaterniond quat_W_B_rqst = rqst_quat*Quaterniond(R_V_B[rqst_id]);
                Quaterniond quat_W_B_rspd = rspd_quat*Quaterniond(R_V_B[rspd_id]);

                rqst_pos = rqst_pos + rqst_quat.toRotationMatrix()*t_V_B[rqst_id];
                rspd_pos = rspd_pos + rspd_quat.toRotationMatrix()*t_V_B[rspd_id];


                double distance = (rqst_pos + quat_W_B_rqst.toRotationMatrix()*rqst_ant_pos - 
                                  (rspd_pos + quat_W_B_rspd.toRotationMatrix()*rspd_ant_pos)).norm();

                static std::vector<uwb_driver::UwbRange> uwb_range_info_msg(nodes_id.size(),
                                                                            uwb_driver::UwbRange());

                if (!range_pub[rqst_idx].getTopic().empty())
                {

                    static random_device rd{};
                    static vector<std::mt19937>
                                loss_gen(nodes_id.size(), mt19937(rd()));
                    static vector<uniform_real_distribution<>>
                                loss_dice(nodes_id.size(),
                                          uniform_real_distribution<>(0, 1.0));

                    static vector<mt19937>
                                dist_err_gen(nodes_id.size(), mt19937(rd()));

                    static vector<normal_distribution<double>>
                                dist_err(nodes_id.size(),
                                         normal_distribution<double>
                                            (range_noise_gaussian[0],
                                             range_noise_gaussian[1]));

                    double loss_dice_value = loss_dice[rqst_idx](loss_gen[rqst_idx]);

                    if(loss_dice_value < 1.0 - loss_chance)
                    {

                        uwb_range_info_msg[rqst_idx].header = std_msgs::Header();
                        uwb_range_info_msg[rqst_idx].header.frame_id = "";
                        uwb_range_info_msg[rqst_idx].header.stamp = nodes_info_msg[rqst_idx].header.stamp;//ros::Time::now();
                        uwb_range_info_msg[rqst_idx].header.seq++;

                        uwb_range_info_msg[rqst_idx].requester_id = rqst_id;
                        uwb_range_info_msg[rqst_idx].requester_idx = rqst_idx;
                        uwb_range_info_msg[rqst_idx].responder_id = rspd_id;
                        uwb_range_info_msg[rqst_idx].responder_idx = rspd_idx;
                        uwb_range_info_msg[rqst_idx].requester_LED_flag = 1;
                        uwb_range_info_msg[rqst_idx].responder_LED_flag = 1;
                        uwb_range_info_msg[rqst_idx].noise = 0;
                        uwb_range_info_msg[rqst_idx].vPeak = 32000;
                        uwb_range_info_msg[rqst_idx].distance = distance + dist_err[rqst_idx](dist_err_gen[rqst_idx]);
                        uwb_range_info_msg[rqst_idx].distance_err = dist_err[rqst_idx](dist_err_gen[rqst_idx]);
                        uwb_range_info_msg[rqst_idx].distance_dot = 0.0;
                        uwb_range_info_msg[rqst_idx].distance_dot_err = 0.0;
                        uwb_range_info_msg[rqst_idx].antenna = ant_rqst_idx << 4 | ant_rspd_idx;
                        uwb_range_info_msg[rqst_idx].stopwatch_time = 123;
                        uwb_range_info_msg[rqst_idx].uwb_time = (uint32_t)(ros::Time::now().toSec()*1000);
                        uwb_range_info_msg[rqst_idx].responder_location.x = rspd_pos(0);
                        uwb_range_info_msg[rqst_idx].responder_location.y = rspd_pos(1);
                        uwb_range_info_msg[rqst_idx].responder_location.z = rspd_pos(2);
                        uwb_range_info_msg[rqst_idx].antenna_offset.x = rqst_ant_pos(0);
                        uwb_range_info_msg[rqst_idx].antenna_offset.y = rqst_ant_pos(1);
                        uwb_range_info_msg[rqst_idx].antenna_offset.z = rqst_ant_pos(2);

                        range_pub[rqst_idx].publish(uwb_range_info_msg[rqst_idx]);

                        printf("Range %d.%d -> %d.%d:\n"
                               "pi = (%.2f, %.2f, %.2f), "
                               "pai = (%.2f, %.2f, %.2f), "
                               "pj = (%.2f, %.2f, %.2f), "
                               "paj = (%.2f, %.2f, %.2f), "
                               "dij = %f, "
                               "dij_sent = %f\n",
                                rqst_idx, ant_rqst_idx,
                                rspd_idx, ant_rspd_idx,
                                rqst_pos(0), rqst_pos(1), rqst_pos(2),
                                rqst_ant_pos(0), rqst_ant_pos(1), rqst_ant_pos(2),
                                rspd_pos(0), rspd_pos(1), rspd_pos(2),
                                rspd_ant_pos(0), rspd_ant_pos(1), rspd_ant_pos(2),
                                distance, uwb_range_info_msg[rqst_idx].distance);
                    }
                }
            }
            else
            {
                printf("Range %d.%d -> %d.%d:\n"
                       "Node position not yet updated. Skipping.\n",
                       rqst_idx, slotmap[slot_idx][j*4 + 2],
                       rspd_idx, slotmap[slot_idx][j*4 + 3]);
            }
        }
    }
    cout << endl;

    double prev_slot_time = slotmap_time[slot_idx];

    // Increment the slot ID
    slot_idx++;
    if (slot_idx == slotmap_time.size())
        slot_idx = 0;

    double curr_slot_time = slotmap_time[slot_idx];

    if (curr_slot_time != prev_slot_time)
    {
        rn_timer.setPeriod(ros::Duration(slotmap_time[slot_idx] - delay_time));

        printf("This slot proc time: %f\n", (ros::Time::now() - proc_start).toSec());

        if(delay_time > slotmap_time[slot_idx] || delay_time < 0)
            delay_time = 0;

        prev_time = ros::Time::now() - ros::Duration(delay_time);
    }
    else
    {
        prev_time = ros::Time::now();
    }

    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rnsim");
    ros::NodeHandle rnsim_nh("~");

//Get all the node IDs-------------------------------------------------------------------------------
    if(rnsim_nh.getParam("nodes_id", nodes_id))
    {
        nodes_total = nodes_id.size();

        printf("Obtained %d nodes' ID:\t", nodes_total);
        for(int i = 0; i < nodes_total - 1; i++)
            printf("%d\t", nodes_id[i]);
        printf("%d\n", nodes_id[nodes_total - 1]);
    }
    else
    {
        printf( "No node ID declared. Exiting...\n" );
        exit(-1);
    }
    cout << endl;
//Get all the node IDs-------------------------------------------------------------------------------


//Get params of the node positions, and create vectors to hold corresponding object------------------
    if(rnsim_nh.getParam("nodes_pos", nodes_pos))
    {
        if (nodes_total == nodes_pos.size()/3)
        {
            printf("Obtained coordinates of %d nodes:\n", nodes_total);
            for(int i = 0; i < nodes_total; i++)
            {
                printf("%3.2f, %3.2f, %3.2f\n", nodes_pos[i*3], nodes_pos[i*3 + 1], nodes_pos[i*3 + 2]);
                antennas_pos.push_back(std::vector<double>{});
                ground_truth_topic.push_back(std::string(""));
                ground_truth_sub.push_back(ros::Subscriber());
                nodes_info_msg.push_back(geometry_msgs::TransformStamped());
                nodes_info_new.push_back(false);

                ground_truth_pub.push_back(ros::Publisher());
                ground_truth_body_pub.push_back(ros::Publisher());

                range_pub.push_back(ros::Publisher());
            }
        }
        else
        {
            printf("Node position is not a triple of IDs. Exiting...\n");
            exit(-2);
        }
    
    }
    else
    {
        printf( "No node position declared. Exit...\n");
        exit(-3);
    }
    cout << endl;
//Get params of the node positions, and create vectors to hold corresponding object------------------


//Get the antenna configurations---------------------------------------------------------------------
    std::vector<double> antennas_pos_;
    if(rnsim_nh.getParam("antennas_pos", antennas_pos_))
    {
        // printf("Obtained antenna positions:\n");
        // for(int i = 0; i < antennas_pos_.size()-1; i++)
        //     printf("%.2f, ", antennas_pos_[i]);
        // printf("%f\t", antennas_pos_[antennas_pos_.size()-1]);

        // printf("Checking validity.\n");
        
        // Check the validity of the declared anchor nodes
        int i = 0;
        while(true)
        {
            int node_id = int(antennas_pos_[i]);
            int node_idx = find_node_idx(node_id);
            if (node_idx != -1)
            {
                int node_antennas = int(antennas_pos_[i+1]);
                for(int j = 0; j < node_antennas; j++)
                {
                    auto x = antennas_pos_[i + 2 + j*3];
                    auto y = antennas_pos_[i + 2 + j*3 + 1];
                    auto z = antennas_pos_[i + 2 + j*3 + 2];
                    // printf("%.2f, %.2f, %.2f\n", x, y, z);

                    antennas_pos[node_idx].push_back(x);
                    antennas_pos[node_idx].push_back(y);
                    antennas_pos[node_idx].push_back(z);
                }
                i = i + 2 + node_antennas*3 + 3;

            }
            else
            {
                // printf("Node ID not found, skipping values until finding a relevant coordinates.\n");
                i++;
            }

            if(i >= antennas_pos_.size())
                break;
        }
    }
    else
    {
        printf("No antenna position declared. All nodes are single-antenna nodes.\n");
    }

    // Set the nodes without any declaration of antennas to zero and display everything.
    for(int i = 0; i < nodes_total; i++)
    {
        auto nodes_antennas = int(antennas_pos[i].size()/3);
        if(nodes_antennas == 0)
        {
            antennas_pos[i].push_back(0);
            antennas_pos[i].push_back(0);
            antennas_pos[i].push_back(0);
            printf("Node %d with 1 antenna:\n%.2f, %.2f, %.2f\n",
                    nodes_id[i], antennas_pos[i][0], antennas_pos[i][1], antennas_pos[i][2]);

        }
        else
        {
            printf("Node %d with %d antennas:\n", nodes_id[i], nodes_antennas);
            for(int j = 0; j < nodes_antennas; j++)
                printf("%.2f, %.2f, %.2f\n",
                        antennas_pos[i][j*3],
                        antennas_pos[i][j*3+1],
                        antennas_pos[i][j*3+2]);
        }
    }
    cout << endl;
//Get the antenna configurations---------------------------------------------------------------------


// Get the body transform----------------------------------------------------------------------------
    for(int i = 0; i < nodes_total; i++)
    {
        R_V_B.insert(make_pair(nodes_id[i], Matrix3d::Identity()));
        t_V_B.insert(make_pair(nodes_id[i], Matrix<double, 3, 1>(0, 0, 0)));
    }
    std::vector<double> T_B_V;
    if(rnsim_nh.getParam("T_B_V", T_B_V))
    {
        const int TF_LEN = 17;
        int tfs = T_B_V.size()/TF_LEN;
        printf("Received transforms for %d node(s).\n", tfs); // 4x4 matrix plus one ID;
        for(int i = 0; i < tfs; i++)
        {
            int node_id = T_B_V[i*TF_LEN];

            Matrix<double, 4, 4> tf = Matrix<double, 4, 4, RowMajor>(&T_B_V[i*TF_LEN + 1]);

            // Transform the T_B_V to T_V_B for conveniences in later calculations
            R_V_B[ node_id ] = tf.block<3, 3>(0, 0).inverse();

            t_V_B[ node_id ] = R_V_B[ node_id ]*(-tf.block<3, 1>(0, 3));

            printf("Node %d, T, R and t:\n", node_id);
            cout << tf << endl;
            cout << R_V_B[ node_id ] << endl;
            cout << t_V_B[ node_id ] << endl;
        }
    }
    else
    {
        printf("No body transform declared. All transforms are indentity\n");
    }
// Get the body transform----------------------------------------------------------------------------




//Get the slot map configuration---------------------------------------------------------------------
    std::vector<double> slotmap_;
    if(rnsim_nh.getParam("slotmap", slotmap_))
    {
        int i = 0;
        while(true)
        {
            int slot_idx = int(slotmap_[i]);

            if(slotmap.size() < slot_idx + 1)
            {
                slotmap.push_back(std::vector<int>{});
                slotmap_time.push_back(slotmap_[i+5]);
            }

            slotmap[slot_idx].push_back(int(slotmap_[i+1]));
            slotmap[slot_idx].push_back(int(slotmap_[i+2]));
            slotmap[slot_idx].push_back(int(slotmap_[i+3]));
            slotmap[slot_idx].push_back(int(slotmap_[i+4]));

            i = i + 6;

            // Skip if there are fewer than 6 values in the array
            if(slotmap_.size() - i < 6)
                break;
        }
    }
    else
    {
        printf("No slot map declared. Exitting.\n");
        exit(-4);
    }

    for(int i = 0; i < slotmap.size(); i++)
    {
        int transactions = slotmap[i].size()/4;
        printf("Slot %d has %d transaction(s):\n", i, transactions);
        // for(int j = 0; j < slotmap[i].size(); j++)
        //     printf("%.0f ", slotmap[i][j]);
        // printf("\n");
        for(int j = 0; j < transactions; j++)
        {
            printf("Time: %.3f, Range %d.%d -> %d.%d\n",
                    slotmap_time[i],
                    slotmap[i][j*4], slotmap[i][j*4 + 2],
                    slotmap[i][j*4 + 1], slotmap[i][j*4 + 3]);
        }
    }
    cout << endl;
//Get the slot map configuration---------------------------------------------------------------------


//Get params of the ground truth topics--------------------------------------------------------------
    std::vector<std::string> ground_truth_topic_;
    if(rnsim_nh.getParam("ground_truth_topic", ground_truth_topic_))
    {
        int i = 0;
        while(true)
        {
            int id = std::stod(ground_truth_topic_[i]);
            auto node_id_it = std::find(std::begin(nodes_id), std::end(nodes_id), id);
            if (node_id_it != std::end(nodes_id))
            {
                auto node_idx = std::distance(nodes_id.begin(), node_id_it);
                ground_truth_topic[node_idx] = ground_truth_topic_[i+1];
                i = i + 2;
            }
            else
            {
                // printf("Node ID not found, skipping values until finding a relevant coordinates.\n");
                i++;
            }

            // Skip if there is fewer than two values in the array
            if(ground_truth_topic_.size() - i < 2)
                break;
        }
    }
    else
    {
        printf("No ground truth declared. Exitting.\n");
        exit(-5);
    }

    // Create the ground truth topic
    for(int i = 0; i < ground_truth_topic.size(); i++)
    {
        if(ground_truth_topic[i].empty())
            printf("Node %d does not have live update.\n", nodes_id[i]);
        else
        {
            printf("Node %d is updated by the topic \"%s\".\n", nodes_id[i], ground_truth_topic[i].c_str());
            ground_truth_sub[i] = rnsim_nh.subscribe<geometry_msgs::TransformStamped>
                                             (ground_truth_topic[i], 100,
                                              boost::bind(&ground_truth_cb, _1, i));
            std::string gt_topic;
            std::stringstream ss;
            ss << nodes_id[i];

            gt_topic = ground_truth_topic[i] + std::string("_ground_truth_") + ss.str();
            ground_truth_pub[i] = rnsim_nh.advertise<geometry_msgs::PoseStamped>(gt_topic, 100);

            gt_topic = ground_truth_topic[i] + std::string("_ground_truth_body_") + ss.str();
            ground_truth_body_pub[i] = rnsim_nh.advertise<geometry_msgs::PoseStamped>(gt_topic, 100);

        }
    }
    cout << endl;
//Get params of the ground truth topics--------------------------------------------------------------


//Create the range topic for requester nodes---------------------------------------------------------
    for(int i = 0; i < slotmap.size(); i++)
    {
        int transactions = slotmap[i].size()/4;
        if (transactions != 0)
        {
            for(int j = 0; j < transactions; j++)
            {
                int rqst_idx = find_node_idx(slotmap[i][j*4]);
                std::stringstream ss;
                ss << nodes_id[rqst_idx];
                if (range_pub[rqst_idx].getTopic().empty())
                    range_pub[rqst_idx] = rnsim_nh.advertise<uwb_driver::UwbRange>(std::string("/uwb_endorange_info_")
                                                                            + ss.str(), 100);
            }
        }
    }
//Create the range topic for requester nodes---------------------------------------------------------

//Create some random generators----------------------------------------------------------------------
    if(rnsim_nh.getParam("loss_chance", loss_chance))
    {
        if (0.0 <= loss_chance && loss_chance < 1.0)
            printf("Ranging loss rate: %d", int(loss_chance*100));
        else
        {
            printf("Ranging loss rate not well-defined. Exitting\n");
            exit(-6);
        }
    }
    else
    {
        printf("Ranging loss rate not declared. Exitting.\n");
        exit(-7);
    }

    if(rnsim_nh.getParam("range_noise_gaussian", range_noise_gaussian))
    {
        printf("Ranging noise declared, mean %f, std: %f.\n",
                range_noise_gaussian[0],
                range_noise_gaussian[1]);
    }
    else
    {
        printf("Ranging noise not declared. Exitting.\n");
        exit(-9);
    }
//Create some random generators----------------------------------------------------------------------
    
    // while(ros::ok())
    // {
        // ros::spinOnce();

        // static int slot_idx = 0;
        // static ros::Time prev_time = ros::Time::now();

        // printf("Slot: %d. Slot true time: %.3f. Intended time: %.3f\n",
        //         slot_idx, (ros::Time::now() - prev_time).toSec(),
        //         slotmap_time[slot_idx]);
        // prev_time = ros::Time::now();
        
        // int transactions = slotmap[slot_idx].size()/4;
        // if (transactions != 0)
        // {
        //     //Calculate the distance and publish
        //     for(int j = 0; j < transactions; j++)
        //     {
        //         printf("Range %d.%d -> %d.%d\n",
        //                 slotmap[slot_idx][j*4], slotmap[slot_idx][j*4 + 2],
        //                 slotmap[slot_idx][j*4 + 1], slotmap[slot_idx][j*4 + 3]);        
        //     }
        // }

        // // Increment the slot ID
        // slot_idx++;
        // if (slot_idx == slotmap_time.size())
        //     slot_idx = 0;

        // ros::Duration(slotmap_time[slot_idx]).sleep();
    // }

    rn_timer = rnsim_nh.createTimer(ros::Duration(slotmap_time[0]), timer_cb);
    ros::spin();

    return 0;
}
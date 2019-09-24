#include "ros/ros.h"
#include <Eigen/Dense>
#include <string>
#include <random>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <geometry_msgs/TransformStamped.h>
// #include <uwb_driver/UwbRange.h>

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

// Slot map
std::vector<std::vector<double>> slot_map;
std::vector<double> slot_map_time;

// Ground truth topic
std::vector<std::string> ground_truth_topic;
std::vector<ros::Subscriber> ground_truth_sub;
std::vector<geometry_msgs::TransformStamped> nodes_info_msg;

std::vector<ros::Publisher> ground_truth_pub;
std::vector<geometry_msgs::PoseStamped> ground_truth_viz_msg;

void ground_truth_cb(const geometry_msgs::TransformStamped::ConstPtr& msg, int i)
{
    // printf("Received update for node %d\n", nodes_id[i]);
    nodes_info_msg[i] = *msg;
    
    ground_truth_viz_msg[i].header = nodes_info_msg[i].header;

    ground_truth_viz_msg[i].pose.position.x = nodes_info_msg[i].transform.translation.x;
    ground_truth_viz_msg[i].pose.position.y = nodes_info_msg[i].transform.translation.y;
    ground_truth_viz_msg[i].pose.position.z = nodes_info_msg[i].transform.translation.z;

    ground_truth_viz_msg[i].pose.orientation.x = nodes_info_msg[i].transform.rotation.x;
    ground_truth_viz_msg[i].pose.orientation.y = nodes_info_msg[i].transform.rotation.y;
    ground_truth_viz_msg[i].pose.orientation.z = nodes_info_msg[i].transform.rotation.z;
    ground_truth_viz_msg[i].pose.orientation.w = nodes_info_msg[i].transform.rotation.w;

    ground_truth_pub[i].publish(ground_truth_viz_msg[i]);

    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rnsim");
    ros::NodeHandle rnsim_nh("~");

//Get all the node IDs
    if(rnsim_nh.getParam("/rnsim/nodes_id", nodes_id))
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
//Get all the node IDs


//Get params of the node positions, and create vectors to hold corresponding object
    if(rnsim_nh.getParam("/rnsim/nodes_pos", nodes_pos))
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

                ground_truth_pub.push_back(ros::Publisher());
                ground_truth_viz_msg.push_back(geometry_msgs::PoseStamped());
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
        exit(-2);
    }
    cout << endl;
//Get params of the node positions, and create vectors to hold corresponding object


//Get the antenna configurations
    std::vector<double> antennas_pos_;
    if(rnsim_nh.getParam("/rnsim/antennas_pos", antennas_pos_))
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
            auto node_id_it = std::find(std::begin(nodes_id), std::end(nodes_id), node_id);
            if (node_id_it != std::end(nodes_id))
            {
                int node_antennas = int(antennas_pos_[i+1]);
                for(int j = 0; j < node_antennas; j++)
                {
                    auto x = antennas_pos_[i + 2 + j*3];
                    auto y = antennas_pos_[i + 2 + j*3 + 1];
                    auto z = antennas_pos_[i + 2 + j*3 + 2];
                    // printf("%.2f, %.2f, %.2f\n", x, y, z);
                    auto node_idx = std::distance(nodes_id.begin(), node_id_it);

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
//Get the antenna configurations


//Get the slot map configuration
    std::vector<double> slot_map_;
    if(rnsim_nh.getParam("/rnsim/slot_map", slot_map_))
    {
        int i = 0;
        while(true)
        {
            int slot_id = int(slot_map_[i]);

            if(slot_map.size() < slot_id + 1)
            {
                slot_map.push_back(std::vector<double>{});
                slot_map_time.push_back(slot_map_[i+5]);
            }

            slot_map[slot_id].push_back(slot_map_[i+1]);
            slot_map[slot_id].push_back(slot_map_[i+2]);
            slot_map[slot_id].push_back(slot_map_[i+3]);
            slot_map[slot_id].push_back(slot_map_[i+4]);

            i = i + 6;

            // Skip if there are fewer than 6 values in the array
            if(slot_map_.size() - i < 6)
                break;
        }
    }
    else
    {
        printf("No slot map declared. Exitting.\n");
        exit(-3);
    }

    for(int i = 0; i < slot_map.size(); i++)
    {
        int transactions = slot_map[i].size()/4;
        printf("Slot %d has %d transaction(s):\n", i, transactions);
        // for(int j = 0; j < slot_map[i].size(); j++)
        //     printf("%.0f ", slot_map[i][j]);
        // printf("\n");
        for(int j = 0; j < transactions; j++)
        {
            printf("Time: %.3f, Range %.0f.%.0f -> %.0f.%.0f\n",
                    slot_map_time[i],
                    slot_map[i][j*4], slot_map[i][j*4 + 2],
                    slot_map[i][j*4 + 1], slot_map[i][j*4 + 3]);
        }
    }
    cout << endl;
//Get the slot map configuration


//Get params of the ground truth topics
    std::vector<std::string> ground_truth_topic_;
    if(rnsim_nh.getParam("/rnsim/ground_truth_topic", ground_truth_topic_))
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
        exit(-4);
    }

    // Create the 
    for(int i = 0; i < ground_truth_topic.size(); i++)
    {
        if(ground_truth_topic[i].empty())
            printf("Node %d does not have live update.\n", nodes_id[i]);
        else
        {
            printf("Node %d is updated by the topic \"%s\".\n", nodes_id[i], ground_truth_topic[i].c_str());
            ground_truth_sub[i] = rnsim_nh.subscribe<geometry_msgs::TransformStamped>
                                             (ground_truth_topic[i], 1,
                                              boost::bind(&ground_truth_cb, _1, i));
            std::string gt_topic;
            std::stringstream ss;
            ss << nodes_id[i];
            gt_topic = ground_truth_topic[i] + std::string("_ground_truth_") + ss.str();

            ground_truth_pub[i] = rnsim_nh.advertise<geometry_msgs::PoseStamped>(gt_topic, 1);
        }
    }
//Get params of the ground truth topics

    ros::spin();

    return 0;
}
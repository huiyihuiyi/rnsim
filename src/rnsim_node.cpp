#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <Eigen/Dense>
#include <string>
#include <random>

#include <geometry_msgs/TransformStamped.h>
// #include <uwb_driver/UwbRange.h>

#include <tf/transform_broadcaster.h>

using namespace std;
using namespace Eigen;

// Listing the IDs of the UWB nodes
std::vector<double> nodes_id;
// // Indices of the UWB nodes as ordered in the param passed to nodes_id
// std::vector<double> nodes_idx;
// Listing the position of the UWB nodes
std::vector<double> nodes_pos;

int nodes_total = 0;

// Listing the position of the antenna node
std::vector<double> antennas_pos_;
std::vector<std::vector<double>> antennas_pos;

// Slot map
std::vector<double> slot_map_;
std::vector<std::vector<double>> slot_map;
std::vector<double> slot_map_time;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "rnsim");
    ros::NodeHandle rnsim_nh("~");

    //Get params of the anchor's ID
    if(rnsim_nh.getParam("/rnsim/nodes_id", nodes_id))
    {
        nodes_total = nodes_id.size();

        printf("Obtained %d nodes' ID:\t", nodes_total);
        for(int i = 0; i < nodes_total - 1; i++)
            printf("%.0f\t", nodes_id[i]);
        printf("%.0f\n", nodes_id[nodes_total - 1]);
    }
    else
    {
        printf( "No node ID declared. Exiting...\n" );
        exit(-1);
    }
    cout << endl;


    //Get params of the anchor
    if(rnsim_nh.getParam("/rnsim/nodes_pos", nodes_pos))
    {
        if (nodes_total == nodes_pos.size()/3)
        {
            printf("Obtained coordinates of %d nodes:\n", nodes_total);
            for(int i = 0; i < nodes_total; i++)
            {
                printf("%3.2f, %3.2f, %3.2f\n", nodes_pos[i*3], nodes_pos[i*3 + 1], nodes_pos[i*3 + 2]);
                antennas_pos.push_back(std::vector<double>{});
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


    //Get params of the antennas
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
            auto node_id_it = std::find(std::begin(nodes_id), std::end(nodes_id), antennas_pos_[i]);
            if (node_id_it != std::end(nodes_id))
            {
                auto node_id = antennas_pos_[i];
                auto node_antennas = int(antennas_pos_[i+1]);
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
            printf("Node %.0f with 1 antenna:\n%.2f, %.2f, %.2f\n",
                    nodes_id[i], antennas_pos[i][0], antennas_pos[i][1], antennas_pos[i][2]);

        }
        else
        {
            printf("Node %.0f with %d antennas:\n", nodes_id[i], nodes_antennas);
            for(int j = 0; j < nodes_antennas; j++)
                printf("%.2f, %.2f, %.2f\n",
                        antennas_pos[i][j*3],
                        antennas_pos[i][j*3+1],
                        antennas_pos[i][j*3+2]);
        }
    }
    cout << endl;

    //Get params of the antennas
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

            if(i >= slot_map_.size())
                break;
        }
    }
    else
    {
        printf("No slot map declared. Exitting.\n");
        exit(-1);
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

    // //Whether to toggle the antenna or not
    // if(rnsim_nh.getParam("/rnsim/toggleAnt", toggleAnt))
    // {
    //     if (toggleAnt)
    //         printf("Toggling antenna selected.\n");
    //     else
    //         printf("Toggling antenna not selected\n");
    // }
    // else
    // {
    //     printf("Antenna toggling not chosen. Exitting...\n");
    //     exit(-3);
    // }

    // std::string vcObjTopic;
    // if(rnsim_nh.getParam("vcObjTopic", vcObjTopic))
    //     printf("vcObjTopic declared: %s\n", vcObjTopic.c_str());
    // else
    //     printf("vcObjTopic not set. No subscriber will be created!\n");

    // std::string vcTarTopic;
    // if(rnsim_nh.getParam("vcTarTopic", vcTarTopic))
    //     printf("vcTarTopic declared: %s\n", vcTarTopic.c_str());
    // else
    //     printf("vcTarTopic not set. No subscriber will be created!\n");


    // range_pub = rnsim_nh.advertise<uwb_driver::UwbRange>("/uwb_endorange_info", 10);
    // ros::Subscriber vicon_sub = rnsim_nh.subscribe("/vicon_xb/viconPoseTopic", 10, vcCallback);
    // ros::Subscriber vic_obj_sub;
    // ros::Subscriber vic_tar_sub;

    // Rtar = MatrixXd::Identity(3, 3);
    // ptar = MatrixXd::Zero(3, 1);

    // if(~vcObjTopic.empty())
    //     vic_obj_sub = rnsim_nh.subscribe(vcObjTopic, 10, vcBridgeObjCallback);

    // if(~vcTarTopic.empty())
    //     vic_tar_sub = rnsim_nh.subscribe(vcObjTopic, 10, vcBridgeTarCallback);


    ros::spin();

    return 0;
}
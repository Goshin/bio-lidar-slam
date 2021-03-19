#pragma clang diagnostic push
#pragma ide diagnostic ignored "performance-unnecessary-value-param"

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <boost/property_tree/ini_parser.hpp>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <pose_cell_network/MappingAction.h>
#include <cognitive_map/Map.h>

#include "utils.h"
#include "experience_map.h"

ExperienceMap *experienceMap = nullptr;
ros::Publisher *mapPublisher = nullptr;
ros::Publisher *estimatedPosePublisher = nullptr;


void publishEstimatedPose() {
    static geometry_msgs::PoseStamped estimatedPose;
    estimatedPose.header.stamp = ros::Time::now();
    estimatedPose.header.seq++;
    estimatedPose.header.frame_id = "odom";
    estimatedPose.pose.position.x = experienceMap->get_experience(experienceMap->get_current_id())->x_m;
    estimatedPose.pose.position.y = experienceMap->get_experience(experienceMap->get_current_id())->y_m;
    estimatedPose.pose.position.z = 0;
    estimatedPose.pose.orientation.x = 0;
    estimatedPose.pose.orientation.y = 0;
    estimatedPose.pose.orientation.z = sin(
            experienceMap->get_experience(experienceMap->get_current_id())->th_rad / 2.0);
    estimatedPose.pose.orientation.w = cos(
            experienceMap->get_experience(experienceMap->get_current_id())->th_rad / 2.0);
    estimatedPosePublisher->publish(estimatedPose);
}

void publishCognitiveMap() {
    static cognitive_map::Map cognitiveMap;
    cognitiveMap.header.stamp = ros::Time::now();
    cognitiveMap.header.seq++;
    cognitiveMap.node_count = experienceMap->get_num_experiences();
    cognitiveMap.node.resize(experienceMap->get_num_experiences());
    for (int i = 0; i < experienceMap->get_num_experiences(); i++) {
        cognitiveMap.node[i].id = experienceMap->get_experience(i)->id;
        cognitiveMap.node[i].pose.position.x = experienceMap->get_experience(i)->x_m;
        cognitiveMap.node[i].pose.position.y = experienceMap->get_experience(i)->y_m;
        cognitiveMap.node[i].pose.orientation.x = 0;
        cognitiveMap.node[i].pose.orientation.y = 0;
        cognitiveMap.node[i].pose.orientation.z = sin(experienceMap->get_experience(i)->th_rad / 2.0);
        cognitiveMap.node[i].pose.orientation.w = cos(experienceMap->get_experience(i)->th_rad / 2.0);
    }

    cognitiveMap.edge_count = experienceMap->get_num_links();
    cognitiveMap.edge.resize(experienceMap->get_num_links());
    for (int i = 0; i < experienceMap->get_num_links(); i++) {
        cognitiveMap.edge[i].source_id = experienceMap->get_link(i)->exp_from_id;
        cognitiveMap.edge[i].destination_id = experienceMap->get_link(i)->exp_to_id;
        cognitiveMap.edge[i].transform.translation.x =
                experienceMap->get_link(i)->d * cos(experienceMap->get_link(i)->heading_rad);
        cognitiveMap.edge[i].transform.translation.y =
                experienceMap->get_link(i)->d * sin(experienceMap->get_link(i)->heading_rad);
        cognitiveMap.edge[i].transform.rotation.x = 0;
        cognitiveMap.edge[i].transform.rotation.y = 0;
        cognitiveMap.edge[i].transform.rotation.z = sin(experienceMap->get_link(i)->facing_rad / 2.0);
        cognitiveMap.edge[i].transform.rotation.w = cos(experienceMap->get_link(i)->facing_rad / 2.0);
    }
    mapPublisher->publish(cognitiveMap);
}

void odometryCallback(nav_msgs::Odometry::ConstPtr odometry) {
    static ros::Time lastTime(0);

    if (lastTime.toSec() == 0) {
        lastTime = odometry->header.stamp;
        return;
    }
    double timeDiff = (odometry->header.stamp - lastTime).toSec();
    experienceMap->on_odo(odometry->twist.twist.linear.x, odometry->twist.twist.angular.z, timeDiff);

    lastTime = odometry->header.stamp;
}

void mappingActionCallback(pose_cell_network::MappingAction::ConstPtr mappingAction) {
    switch (mappingAction->action) {
        case pose_cell_network::MappingAction::CREATE_NODE:
            experienceMap->on_create_experience(mappingAction->dest_id);
            experienceMap->on_set_experience(mappingAction->dest_id, 0);
            break;

        case pose_cell_network::MappingAction::CREATE_EDGE:
            experienceMap->on_create_link(mappingAction->src_id, mappingAction->dest_id, mappingAction->relative_rad);
            experienceMap->on_set_experience(mappingAction->dest_id, mappingAction->relative_rad);
            break;

        case pose_cell_network::MappingAction::SET_NODE:
            ROS_ERROR("em->on_set_experience");
            experienceMap->on_set_experience(mappingAction->dest_id, mappingAction->relative_rad);
            break;
    }

    experienceMap->iterate();
    publishEstimatedPose();

    static ros::Time lastTime(0);
    if (mappingAction->header.stamp - lastTime > ros::Duration(1.0)) {
        lastTime = mappingAction->header.stamp;
        publishCognitiveMap();
    }

}


int main(int argc, char *argv[]) {
    if (argc < 2) {
        ROS_FATAL("Need a config file");
        exit(-1);
    }
    boost::property_tree::ptree settings, cognitiveMapSettings;
    read_ini(argv[1], settings);
    get_setting_child(cognitiveMapSettings, settings, "cognitive_map", true);

    ros::init(argc, argv, "cognitive_map");
    ros::NodeHandle nh;

    experienceMap = new ExperienceMap(cognitiveMapSettings);

    mapPublisher = new ros::Publisher(nh.advertise<cognitive_map::Map>("/cognitive_map", 1));
    estimatedPosePublisher = new ros::Publisher(nh.advertise<geometry_msgs::PoseStamped>("/estimated_pose", 1));

    ros::Subscriber odometrySubscriber = nh.subscribe<nav_msgs::Odometry>("/odom", 0, odometryCallback);
    ros::Subscriber mappingActionSubscriber = nh.subscribe<pose_cell_network::MappingAction>(
            "/pose_cell_network_mapping_action",
            0, mappingActionCallback);

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin();

    return 0;
}

#pragma clang diagnostic pop

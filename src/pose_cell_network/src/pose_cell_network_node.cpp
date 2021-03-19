#pragma clang diagnostic push
#pragma ide diagnostic ignored "performance-unnecessary-value-param"

#include <ros/ros.h>
#include <boost/property_tree/ini_parser.hpp>
#include <nav_msgs/Odometry.h>
#include <lidar_local_view_cell/LiDARViewTemplate.h>
#include <pose_cell_network/MappingAction.h>
#include "posecell_network.h"
#include "utils.h"

PosecellNetwork *poseCellNetwork = nullptr;
ros::Publisher *mappingActionPublisher = nullptr;

void lidarViewTemplateCallback(lidar_local_view_cell::LiDARViewTemplate::ConstPtr viewTemplate) {
    poseCellNetwork->on_view_template(viewTemplate->view_id);
}

void publishMappingAction() {
    static pose_cell_network::MappingAction mappingAction;
    mappingAction.src_id = poseCellNetwork->get_current_exp_id();
    mappingAction.action = poseCellNetwork->get_action();
    if (mappingAction.action != PosecellNetwork::NO_ACTION) {
        mappingAction.header.stamp = ros::Time::now();
        mappingAction.header.seq++;
        mappingAction.dest_id = poseCellNetwork->get_current_exp_id();
        mappingAction.relative_rad = poseCellNetwork->get_relative_rad();
        mappingActionPublisher->publish(mappingAction);
    }
}

void odometryCallback(nav_msgs::Odometry::ConstPtr odometry) {
    static ros::Time lastMsgTime(0);

    if (lastMsgTime.toSec() == 0) {
        lastMsgTime = odometry->header.stamp;
        return;
    }
    double timeDiff = (odometry->header.stamp - lastMsgTime).toSec();
    poseCellNetwork->on_odo(odometry->twist.twist.linear.x, odometry->twist.twist.angular.z, timeDiff);
    publishMappingAction();

    lastMsgTime = odometry->header.stamp;
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        ROS_FATAL("Need a config file");
        exit(-1);
    }

    boost::property_tree::ptree settings, poseCellSettings;
    read_ini(argv[1], settings);
    get_setting_child(poseCellSettings, settings, "pose_cell", true);

    ros::init(argc, argv, "pose_cell_network");
    ros::NodeHandle nh;

    poseCellNetwork = new PosecellNetwork(poseCellSettings);
    mappingActionPublisher = new ros::Publisher(
            nh.advertise<pose_cell_network::MappingAction>("/pose_cell_network_mapping_action", 0));

    ros::Subscriber odometrySubscriber = nh.subscribe<nav_msgs::Odometry>("/odom", 0, odometryCallback);
    ros::Subscriber templateSubscriber = nh.subscribe<lidar_local_view_cell::LiDARViewTemplate>("/lidar_local_view",
                                                                                                0,
                                                                                                lidarViewTemplateCallback);

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin();

    return 0;
}

#pragma clang diagnostic pop
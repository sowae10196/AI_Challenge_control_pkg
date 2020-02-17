#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <istream>
#include <math.h>

#define NUMBER_OF_WAYPOINT 15
#define PI 3.141592

geometry_msgs::PoseStamped local_pose;
geometry_msgs::PoseStamped waypoint_;

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void waypoint_init(double (*waypoint_array)[4]);
double calculate_distance();

bool start = false;

int main(int argc, char** argv){
    ros::init(argc, argv, "local_waypoint_publisher");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    ros::Publisher local_waypoint_pub = nh.advertise<geometry_msgs::PoseStamped>("waypoint_", 10);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("mavros/local_position/pose", 100, pose_cb);
    // std::ifstream();
    double waypoint[NUMBER_OF_WAYPOINT][4];
    int index = 0;
    float x = 0;
    float y = 0;
    float z = 0;
    float x_ = 0;
    float y_ = 0;
    float z_ = 0;
    float dist = 0;
    float theta = 0;
    float orient_x = 0;
    float orient_y = 0;
    float orient_z = 0;
    float orient_w = 0;
    tf::Quaternion myQuaternion;

    do{
        ros::spinOnce();
        loop_rate.sleep();
    }while(!start);

    ROS_INFO_STREAM("Publishing Waypoint!");
    waypoint_init(waypoint);

    while(ros::ok()){
        x = local_pose.pose.position.x;
        y = local_pose.pose.position.y;
        z = local_pose.pose.position.z;
        x_ = waypoint[index][0];
        y_ = waypoint[index][1];
        z_ = waypoint[index][2];
        myQuaternion.setRPY(0, 0, (waypoint[index][3] / (18 * PI)));
        orient_x = myQuaternion.x();
        orient_y = myQuaternion.y();
        orient_z = myQuaternion.z();
        orient_w = myQuaternion.w();

        dist = sqrt((x - x_) * (x - x_) + (y - y_) * (y - y_) + (z - z_) * (z - z_));
        
        if(fabs(dist) < 0.3)    index++;
        if(index >= NUMBER_OF_WAYPOINT) break;

        waypoint_.pose.position.x = waypoint[index][0];
        waypoint_.pose.position.y = waypoint[index][1];
        waypoint_.pose.position.z = waypoint[index][2];
        waypoint_.pose.orientation.x = orient_x;
        waypoint_.pose.orientation.y = orient_y;
        waypoint_.pose.orientation.z = orient_z;
        waypoint_.pose.orientation.w = orient_w;
        local_waypoint_pub.publish(waypoint_);

        ROS_INFO_STREAM(dist);
        ROS_INFO_STREAM(index);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    start = true;
    local_pose = *msg;
}

void waypoint_init(double (*waypoint_array)[4]){
    waypoint_array[0][0] = 0;
    waypoint_array[0][1] = 0;
    waypoint_array[0][2] = 2;
    waypoint_array[0][3] = -90;

    waypoint_array[1][0] = 0;
    waypoint_array[1][1] = 2;
    waypoint_array[1][2] = 2;
    waypoint_array[1][3] = -90;

    waypoint_array[2][0] = 3;
    waypoint_array[2][1] = 2;
    waypoint_array[2][2] = 2;
    waypoint_array[2][3] = -90;

    waypoint_array[3][0] = 3;
    waypoint_array[3][1] = 6;
    waypoint_array[3][2] = 2;
    waypoint_array[3][3] = -90;

    waypoint_array[4][0] = 0;
    waypoint_array[4][1] = 6;
    waypoint_array[4][2] = 2;
    waypoint_array[4][3] = -90;

    waypoint_array[5][0] = 0;
    waypoint_array[5][1] = 12;
    waypoint_array[5][2] = 2;
    waypoint_array[5][3] = -90;

    waypoint_array[6][0] = 1;
    waypoint_array[6][1] = 12;
    waypoint_array[6][2] = 2;
    waypoint_array[6][3] = -90;

    waypoint_array[7][0] = 1;
    waypoint_array[7][1] = 16;
    waypoint_array[7][2] = 2;
    waypoint_array[7][3] = -90;

    waypoint_array[8][0] = 0;
    waypoint_array[8][1] = 16;
    waypoint_array[8][2] = 2;
    waypoint_array[8][3] = -90;

    waypoint_array[9][0] = 0;
    waypoint_array[9][1] = 25.5;
    waypoint_array[9][2] = 2;
    waypoint_array[9][3] = -90;

    waypoint_array[10][0] = 0;
    waypoint_array[10][1] = 25.5;
    waypoint_array[10][2] = 1;
    waypoint_array[10][3] = -180;

    waypoint_array[11][0] = 11;
    waypoint_array[11][1] = 25.5;
    waypoint_array[11][2] = 1;
    waypoint_array[11][3] = -180;

    waypoint_array[12][0] = 11;
    waypoint_array[12][1] = 25.5;
    waypoint_array[12][2] = 2;
    waypoint_array[12][3] = -270;

    waypoint_array[13][0] = 10;
    waypoint_array[13][1] = 25.5;
    waypoint_array[13][2] = 2;
    waypoint_array[13][3] = -270;

    waypoint_array[14][0] = 10;
    waypoint_array[14][1] = 0;
    waypoint_array[14][2] = 2;
    waypoint_array[14][3] = -270;

}
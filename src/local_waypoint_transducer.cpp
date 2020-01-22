#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "local_waypoint_transducer");
    ros::NodeHandle nh;

    ros::Publisher local_waypoint_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("test_", 10);
    ros::Rate loop_rate(10);
    geometry_msgs::PoseStamped msg;

    while(ros::ok()){
        msg.pose.position.x = 0;
        msg.pose.position.y = 1;
        msg.pose.position.z = 1;

        local_waypoint_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
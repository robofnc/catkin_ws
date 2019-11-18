#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv){
    ros::init(argc,argv,"pub");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter",10);
    ros::Rate loop_rate(10);

    while(ros::ok()){
        std_msgs::String out;
        out.data = "Yukiho wa Kawaii!";
        ROS_INFO("%s",out.data.c_str());
        chatter_pub.publish(out);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
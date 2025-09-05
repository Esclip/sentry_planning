#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>

/* mbot速度消息发布器 */
ros::Publisher mbotLeftSpeedPublisher;
ros::Publisher mbotRightSpeedPublisher;

int main (int argc, char** argv) {
                        /** 初始化 **/
    /* 注册节点 */
    ros::init(argc, argv, "mbot_controller");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    /* 消息发布器初始化 */
    mbotLeftSpeedPublisher = n.advertise<std_msgs::Float64>("/mbot/left_wheel_joint_controller/command", 1);
    mbotRightSpeedPublisher = n.advertise<std_msgs::Float64>("/mbot/right_wheel_joint_controller/command", 1);

    /* 目标速度更新周期设置为1ms*/
    ros::Rate loop_rate(1000);
    while (1) {
        std_msgs::Float64 left_speed;
        std_msgs::Float64 right_speed;
        left_speed.data = 10.0;
        right_speed.data = 10.0;
        mbotLeftSpeedPublisher.publish(left_speed);
        mbotRightSpeedPublisher.publish(right_speed);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

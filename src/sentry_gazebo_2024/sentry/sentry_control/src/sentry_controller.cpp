#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>

/* mbot速度消息发布器 */
ros::Publisher sentrySpinLeftBackPostiionPublisher;
ros::Publisher sentrySpinRightBackPostiionPublisher;
ros::Publisher sentrySpinLeftFrontPostiionPublisher;
ros::Publisher sentrySpinRightFrontPostiionPublisher;
ros::Publisher sentryWheelLeftBackSpeedPublisher;
ros::Publisher sentryWheelRightBackSpeedPublisher;
ros::Publisher sentryWheelLeftFrontSpeedPublisher;
ros::Publisher sentryWheelRightFrontSpeedPublisher;

int main (int argc, char** argv) {
                        /** 初始化 **/
    /* 注册节点 */
    ros::init(argc, argv, "sentry_controller");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    /* 消息发布器初始化 */
    sentrySpinLeftBackPostiionPublisher = n.advertise<std_msgs::Float64>("/sentry/spin_leftback_joint_controller/command", 1);
    sentrySpinRightBackPostiionPublisher = n.advertise<std_msgs::Float64>("/sentry/spin_rightback_joint_controller/command", 1);
    sentrySpinLeftFrontPostiionPublisher = n.advertise<std_msgs::Float64>("/sentry/spin_leftfront_joint_controller/command", 1);
    sentrySpinRightFrontPostiionPublisher = n.advertise<std_msgs::Float64>("/sentry/spin_rightfront_joint_controller/command", 1);
    sentryWheelLeftBackSpeedPublisher = n.advertise<std_msgs::Float64>("/sentry/wheel_leftback_joint_controller/command", 1);
    sentryWheelRightBackSpeedPublisher = n.advertise<std_msgs::Float64>("/sentry/wheel_rightback_joint_controller/command", 1);
    sentryWheelLeftFrontSpeedPublisher = n.advertise<std_msgs::Float64>("/sentry/wheel_leftfront_joint_controller/command", 1);
    sentryWheelRightFrontSpeedPublisher = n.advertise<std_msgs::Float64>("/sentry/wheel_rightfront_joint_controller/command", 1);

    /* 目标速度更新周期设置为1ms*/
    ros::Rate loop_rate(1000);
    while (1) {
        std_msgs::Float64 SpinLeftBackPositon;
        std_msgs::Float64 SpinRightBackPositon;
        std_msgs::Float64 SpinLeftFrontPositon;
        std_msgs::Float64 SpinRightFrontPositon;
        std_msgs::Float64 WheelLeftBackSpeed;
        std_msgs::Float64 WheelRightBackSpeed;
        std_msgs::Float64 WheelLeftFrontSpeed;
        std_msgs::Float64 WheelRightFrontSpeed;
        SpinLeftBackPositon.data = 0;
        SpinRightBackPositon.data = 0;
        SpinLeftFrontPositon.data = 0;
        SpinRightFrontPositon.data = 0;
        WheelLeftBackSpeed.data = 1;
        WheelRightBackSpeed.data = 1;
        WheelLeftFrontSpeed.data = 1;
        WheelRightFrontSpeed.data = 1;
        sentrySpinLeftBackPostiionPublisher.publish(SpinLeftBackPositon);
        sentrySpinRightBackPostiionPublisher.publish(SpinRightBackPositon);
        sentrySpinLeftFrontPostiionPublisher.publish(SpinLeftFrontPositon);
        sentrySpinRightFrontPostiionPublisher.publish(SpinRightFrontPositon);
        sentryWheelLeftBackSpeedPublisher.publish(WheelLeftBackSpeed);
        sentryWheelRightBackSpeedPublisher.publish(WheelRightBackSpeed);
        sentryWheelLeftFrontSpeedPublisher.publish(WheelLeftFrontSpeed);
        sentryWheelRightFrontSpeedPublisher.publish(WheelRightFrontSpeed);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

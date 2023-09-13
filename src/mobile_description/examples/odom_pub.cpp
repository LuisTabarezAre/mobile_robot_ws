#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"

class MyOdometryNode: public rclcpp::Node
{
    public:
        MyOdometryNode() : Node("odometry_publisher")
        {
            publisher_=this->create_publisher<nav_msgs::msg::Odometry>("mobile_odom",10);
            //timer_ = this->create_wall_timer();
        }
    private:
        void timer_callback()
        {
            auto current_time = rclcpp::Clock(RCL_ROS_TIME).now();

            double delta_x= 0.0;
            double delta_y=0.0;
            double delta_th=0.0;

            auto last_time=current_time;

        }
        double x=0.0;
        double y=0.0;
        double th=0.0;

        double vx=0.0;
        double vy=0.0;
        double vth=0.0;

        rclcpp::Clock(RCL_ROS_TIME);

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<MyOdometryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
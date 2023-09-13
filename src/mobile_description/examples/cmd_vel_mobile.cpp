#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <cmath>


class MyMobilleCmdVel: public rclcpp::Node
{
    public:
    MyMobilleCmdVel():Node("cmd_vel_mobile_controller"),mobil_up_(false)
    {
        target_x=5.0;
        target_y=5.0;

        publisher_=this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
        subscriber_=this->create_subscription<nav_msgs::msg::Odometry>("/odom",10,std::bind(&MyMobilleCmdVel::callback_pose,this,std::placeholders::_1));
        timer_=this->create_wall_timer(std::chrono::milliseconds(20),std::bind(&MyMobilleCmdVel::control_loop,this));
    }
    private:
        void callback_pose(const nav_msgs::msg::Odometry::SharedPtr pose)
        {
            pose_ = *pose.get();
            mobil_up_ = true;
        }
        void control_loop()
        {
            if(!mobil_up_){
                return;
            }

            //* Message
            auto msg=geometry_msgs::msg::Twist();

            //* Position calculus
            double distance_x=target_x-pose_.pose.pose.position.x;
            double distance_y=target_y-pose_.pose.pose.position.y;
            double position_error=std::sqrt(distance_x * distance_x + distance_y * distance_y);

            //* Orientation calculus
            //* yaw (z-axis rotation)
            double siny_cosp=2*(pose_.pose.pose.orientation.w * pose_.pose.pose.orientation.z +
                                pose_.pose.pose.orientation.x * pose_.pose.pose.orientation.y);
            double cosy_cosp=1 - 2*(pose_.pose.pose.orientation.y * pose_.pose.pose.orientation.y +
                                    pose_.pose.pose.orientation.z * pose_.pose.pose.orientation.z);

            double yaml=std::atan2(siny_cosp,cosy_cosp);
            double desired_theta = 0.0;
            double orientation_error = 0.0;

            if (position_error>0.10)
            {
                msg.linear.x=0.5;
                
                //* Orientation
                desired_theta = std::atan2(distance_y,distance_x);
                orientation_error = desired_theta-yaml;

                //* Bound error
                orientation_error = std::atan2(std::sin(orientation_error),std::cos(orientation_error));

                msg.angular.z = 1*orientation_error - 0.5*pose_.twist.twist.angular.z;

            }
            else if(position_error<=0.10 && position_error>0.025)
            {
                //* Positon
                msg.linear.x=0.30*position_error;
                msg.angular.z = 0.30*position_error;
            }
            else
            {
                msg.linear.x=0.0;
                msg.angular.z=0.0;
            }
            //std::cout<<yaml<<" , "<<position_error<<" , "<<orientation_error<<std::endl;
            publisher_->publish(msg);
        }

        double target_x, target_y;
        bool mobil_up_;
        
        nav_msgs::msg::Odometry pose_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int args,char * argv [])
{
    rclcpp::init(args,argv);
    auto node = std::make_shared<MyMobilleCmdVel>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

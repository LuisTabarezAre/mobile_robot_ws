#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class MyCmdJointTorque : public rclcpp::Node
{
    public:
        MyCmdJointTorque(): Node("cmd_joint_torque"), mobile_up_joint(false)
        {
            this->set_parameter(rclcpp::Parameter("use_sim_time",true));
            publisher_=this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controllers/commands",10);
            subscriber_=this->create_subscription<sensor_msgs::msg::JointState>("/joint_states",10,std::bind(&MyCmdJointTorque::callback_joint_state,this,std::placeholders::_1));
            timer_=this->create_wall_timer(std::chrono::milliseconds(2),std::bind(&MyCmdJointTorque::controll_loop,this));
        }
    private:
        void callback_joint_state(const sensor_msgs::msg::JointState::SharedPtr joint_state)
        {
            joints_ = *joint_state.get();
            mobile_up_joint = true;
        }
        void controll_loop()
        {
            auto tau=std_msgs::msg::Float64MultiArray();
            std::vector<std::string> joint_names = {"joint_l","joint_r"};
            tau.data.resize(joint_names.size());

            // Command Joint Torque
            tau.data[0]=1.1;
            tau.data[1]=1.1;

            publisher_->publish(tau);
        }
        sensor_msgs::msg::JointState joints_;

        bool mobile_up_joint;

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int args, char * argv[])
{
    rclcpp::init(args,argv);
    auto node = std::make_shared<MyCmdJointTorque>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
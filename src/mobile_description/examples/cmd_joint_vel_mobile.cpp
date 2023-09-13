#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <vector>
#include <chrono>
#include <cmath>

class MyCmdJointVel: public rclcpp::Node
{
    public:
        MyCmdJointVel():Node("cmd_joint_vel"),mobile_up_joint(false)
        {
            b=0.28341;
            r=0.19435/2.0;
            x=0.0;
            y=0.0;
            theta=0.0;
            last_time=0.0;

            target_x=-5.0;
            target_y=-5.0;

            old_orientation_error=0.0;

            this->set_parameter(rclcpp::Parameter("use_sim_time",true));
            publisher_=this->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_controller/commands",10);
            subscriber_joint_state_=this->create_subscription<sensor_msgs::msg::JointState>("/joint_states",10,
                                                                std::bind(&MyCmdJointVel::callback_joint_state,this,std::placeholders::_1));
            subscriber_imu_=this->create_subscription<sensor_msgs::msg::Imu>("/demo/imu",10,
                                                                std::bind(&MyCmdJointVel::callback_IMU,this,std::placeholders::_1));
            
            timer_ = this->create_wall_timer(std::chrono::milliseconds(2),std::bind(&MyCmdJointVel::control_loop,this));
        }
       
    private:
        void callback_joint_state(const sensor_msgs::msg::JointState::SharedPtr state)
        {
            state_ = *state.get();
            mobile_up_joint = true;
        }
        void callback_IMU(const sensor_msgs::msg::Imu::SharedPtr imu)
        {
            imu_=*imu.get();
        }

        void control_loop()
        {
            if(!mobile_up_joint) return;

            auto vel=std_msgs::msg::Float64MultiArray();
            std::vector<std::string> joint_names = {"q1","q2"};
            std::vector<double> rpy_={0.0,0.0,0.0};

            rpy_=RPY_Orientation(imu_.orientation.x,imu_.orientation.y,imu_.orientation.z,imu_.orientation.w);

            //* Time variables
            double current_time=now().seconds();
            double  seconds_=current_time-last_time;
            last_time=current_time;

            //* State Estimation
            odom_estimation(seconds_,x,y,rpy_.at(2),b,state_.velocity[0],state_.velocity[1],r);
            vel.data.resize(joint_names.size());

            RCLCPP_INFO_STREAM(get_logger(),"now().seconds(): " << now().seconds());
            RCLCPP_INFO_STREAM(get_logger(),"[Pos_x: "<< x << " ] " << "[Pos_y: "<< y << " ] " <<
                                            "[Roll: " <<rpy_.at(0)<< " ] " << "[Pitch: " <<rpy_.at(1)<< " ] " << "[Yaml: " <<rpy_.at(2)<< " ] ");

            // Control 
            // * Position calculus
            double distance_x=target_x-x;
            double distance_y=target_y-y;
            double position_error=std::sqrt(distance_x * distance_x + distance_y * distance_y);
            
            double desired_theta = 0.0;
            double orientation_error = 0.0;

            // Velocities
            double v=0.0;
            double w=0.0;

            if(position_error>0.10)
            {
                v=0.5;

                // Orientation
                desired_theta = std::atan2(distance_y,distance_x);
                orientation_error = desired_theta-rpy_.at(2);

                // Bound error
                orientation_error = std::atan2(std::sin(orientation_error),std::cos(orientation_error));

                w=0.3*orientation_error-0.1*imu_.angular_velocity.z;
            }

            else if(position_error<=0.10 && position_error>0.025)
            {
                v=0.10*position_error;
                w=0.10*position_error;
            }

            else
            {
                v=0;
                w=0;
            }
            old_orientation_error= orientation_error;
            double vl=(2.0*v-w*b)/(2.0*r);
            double vr=(2.0*v+w*b)/(2.0*r);

            // Command Joint Velocity
            vel.data[0]=vl;
            vel.data[1]=vr;

            publisher_->publish(vel); 
        }

        void odom_estimation(double last_update, double & x, double & y, double & theta, double b,double vl, double vr,double r)
        {
            double sl= vl*r*last_update;
            double sr= vr*r*last_update;
            
            double ssum=sl+sr;
            double sdiff=sr-sl;

            double dx=(ssum/2.0)*(std::cos((theta)+(sdiff)/(2.0*b)));
            double dy=(ssum/2.0)*(std::sin((theta)+(sdiff)/(2.0*b)));
            //double dtheta=sdiff/b;
    
            x+=(dx);
            y+=(dy);
            //theta+=dtheta;
        }

        std::vector<double> RPY_Orientation(double x,double y,double z,double w)
        {
            double roll=0.0;
            double pitch=0.0;
            double yaml=0.0;

            //* Orientation calculus

            //* Roll (x-axis rotation)
            double sinr_cosp=2*(w*x + y*z);
            double cosr_cosp=1 - 2*(x*x + y*y);
            roll=std::atan2(sinr_cosp,cosr_cosp);

            //* Pitch (y-axis rotation)
            double sinp=std::sqrt(1 + 2*(w*y + x*z));
            double cosp=std::sqrt(1 - 2*(w*y + x*z));
            pitch=2.0*std::atan2(sinp,cosp)- M_PI/2.0;

            //* Yaw (z-axis rotation)
            double siny_cosp=2*(w*z + x*y);
            double cosy_cosp=1 - 2*(y*y + z*z);
            yaml=std::atan2(siny_cosp,cosy_cosp);

            std::vector<double> RPY={0.0,0.0,0.0};

            RPY.at(0)=roll;
            RPY.at(1)=pitch;
            RPY.at(2)=yaml;

            return RPY;
        }

        double b; //Distance between wheels
        double r; //Wheels radius
        double x; //Mobile Position in x coordinate 
        double y; //Mobile Position in y coordinate
        double theta; //Mobile Orientation in Yaw (z-axis)
        double last_time;
        double old_orientation_error;

        double target_x;
        double target_y;

        bool mobile_up_joint;

        sensor_msgs::msg::JointState state_;
        sensor_msgs::msg::Imu imu_;

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_joint_state_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_imu_;

        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int args, char * argv[])
{
    rclcpp::init(args,argv);
    auto node = std::make_shared<MyCmdJointVel>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
// PID controller plan-ish 
// - position controller that works by changing x,y,z velocity and yaw rate 
// - use error between position in mocap frame (local in sim) and desired position in same frame -> multiplied by Kp
// - integral of error multiplied by Ki -> each time step integral incremented by error*dt
// - derviative of error can also be included and multiplied by Kd -> leave for now
// - create new message type to handle this. Will make code a lot cleaner 
// - Gains should be tuneable by rqt dynmaic reconfigure 

// Controller should always use latest actual position data (assuming reference will be relatively constant)
// Controller outputs can be based on a pose subscriber, a second subscriber can change the reference values in the class (should allow for a higher rate)
#include <ros/ros.h>

#include <Eigen/Core>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/State.h>

#include <slapper_control/pid_controllerConfig.h>
#include <dynamic_reconfigure/server.h>

//Global variable so these can be dynamically reconfigured
double kp_x, kp_y, kp_z, kp_yaw;                            // Proportional gains
double ki_x, ki_y, ki_z, ki_yaw;                            // Integral gains
double kd_x, kd_y, kd_z, kd_yaw;                            // Derivative gains
std::string reference_topic = "/mocap_position_setpoint";   // Topic through which the reference (desired) states will be passed
std::string pose_topic = "/mocap/slapper";                  // Topic with real-time drone pose

class PositionPID{
    public:
        PositionPID(){
            reference_state_received_ = false;
            state_received_ = false;

            last_error_ << 0.0, 0.0, 0.0, 0.0;
            integral_ << 0.0, 0.0, 0.0, 0.0;

            control_loop_ = nH_.subscribe(pose_topic, 10, &PositionPID::control_loop_cb, this);
            reference_sub_ = nH_.subscribe(reference_topic, 10, &PositionPID::reference_cb, this);
            }

        void control_loop_cb(const geometry_msgs::PoseStamped::ConstPtr pose_msg){

            double drone_roll, drone_pitch, drone_yaw;
            tf2::Quaternion pose_q(pose_msg->pose.orientation.x,
                                   pose_msg->pose.orientation.y,
                                   pose_msg->pose.orientation.z,
                                   pose_msg->pose.orientation.w);
            tf2::Matrix3x3 pose_matrix(pose_q);
            pose_matrix.getRPY(drone_roll, drone_pitch, drone_yaw);

            state_vector_ << pose_msg->pose.position.x, 
                             pose_msg->pose.position.y,
                             pose_msg->pose.position.z,
                             drone_yaw;

            state_received_ = true


        }

        void reference_cb(const geometry_msgs::PoseStamped::ConstPtr ref_pose_msg){
            double ref_roll, ref_pitch, ref_yaw;
            tf2::Quaternion pose_q(ref_pose_msg->pose.orientation.x, 
                                   ref_pose_msg->pose.orientation.y,
                                   ref_pose_msg->pose.orientation.z,
                                   ref_pose_msg->pose.orientation.w);
            tf2::Matrix3x3 pose_matrix(pose_q);
            pose_matrix.getRPY(ref_roll, ref_pitch, ref_yaw);
            
            reference_state_ << ref_pose_msg->pose.position.x,
                                ref_pose_msg->pose.position.y,
                                ref_pose_msg->pose.position.z,
                                ref_yaw;

            reference_state_received_ = true;
        }

        bool reference_state_received(){
            return reference_state_received_;
        }

        bool state_received(){
            return state_received_;
        }

        void reset(){
            last_error_ << 0.0, 0.0, 0.0, 0.0;
            integral_ << 0.0, 0.0, 0.0, 0.0;
            ROS_INFO("Controller reset");
        }

        Eigen::Array4d calculate_control_outputs(double dt) {
            Eigen::Array4d error = reference_state_ - state_vector_;
            
            Eigen::Array4d kp(kp_x, kp_y, kp_z, kp_yaw);          
            Eigen::Array4d ki(ki_x, ki_y, ki_z, ki_yaw); 
            Eigen::Array4d kd(kd_x, kd_y, kd_z, kd_yaw);

            Eigen::Array4d proportional_term = kp * error;
            integral_ += ki * error * dt; 
            Eigen::Array4d derivative_term = kd * (error - last_error_) / dt;    

            last_error_ = error;

            Eigen::Array4d output = proportional_term + integral_ + derivative_term;
            

            return output;
        }

    private:
        bool reference_state_received_;         // Flag indicating whether the first reference state has been received
        bool state_received_;                   // Flag indicating whether actual vehicle state has been received (for safety)

        Eigen::Array4d last_error_;             // Error from the previous calculation
        Eigen::Array4d integral_;               // Accumulated error over time
        
        Eigen::Array4d state_vector_;           // Drone state vector
        Eigen::Array4d reference_state_;        // Reference state values

        ros::NodeHandle nH_;                    // ROS node handle
        ros::Subscriber control_loop_;          // Control loop subscriber, subscribes to drone pose
        ros::Subscriber reference_sub_;         // Reference subscriber, subscribers to reference variable stream
};

bool drone_armed = false;
void state_cb(const mavros_msgs::State::ConstPtr& state_msg){
    drone_armed = state_msg->armed;
}

//define dynamic reconfigure server callback function
void callback(slapper_control::pid_controllerConfig &config, uint32_t level) {
  kp_x = config.kp_x;
  kp_y = config.kp_y;
  kp_z = config.kp_z;
  kp_yaw = config.kp_yaw;
  ki_x = config.ki_x;
  ki_y = config.ki_y;
  ki_z = config.ki_z;
  ki_yaw = config.ki_yaw;
  kd_x = config.kd_x;
  kd_y = config.kd_y;
  kd_z = config.kd_z;
  kd_yaw = config.kd_yaw;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid_position_controller_node");
    ros::NodeHandle nh;

    //Declare objects for dynamic parameter reconfiguration server and callback with config type
    dynamic_reconfigure::Server<slapper_control::pid_controllerConfig> server;
    dynamic_reconfigure::Server<slapper_control::pid_controllerConfig>::CallbackType f;

    //Set it so that when the server gets a reconfigure request it calls the callback function
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    // Initially set paramater values to those values passed in launch file
    nh.getParam("/pid_controller/kp_x", kp_x);
    nh.getParam("/pid_controller/kp_y", kp_y);
    nh.getParam("/pid_controller/kp_z", kp_z);
    nh.getParam("/pid_controller/kp_yaw", kp_yaw);
    nh.getParam("/pid_controller/ki_x", ki_x);
    nh.getParam("/pid_controller/ki_y", ki_y);
    nh.getParam("/pid/_controllerki_z", ki_z);
    nh.getParam("/pid_controller/ki_yaw", ki_yaw);
    nh.getParam("/pid_controller/kd_x", kd_x);
    nh.getParam("/pid_controller/kd_y", kd_y);
    nh.getParam("/pid_controller/kd_z", kd_z);
    nh.getParam("/pid_controller/kd_yaw", kd_yaw);

    nh.getParam("/pid_controller/reference_topic", reference_topic);
    nh.getParam("/pid_controller/pose_topic", pose_topic);

    ros::Publisher velocity_command_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    //subscribe to state to check for arming
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    
    ros::Rate rate(20.0);

    // Wait 5 seconds before starting the PID controller so that the controller only starts after setpoints are being published
    for(int i = 100; ros::ok() && i > 0; --i){
        ros::spinOnce();
        rate.sleep();
    }

    PositionPID position_controller;
    bool initial_arming = false;

    double last_loop_time = ros::Time::now().toSec();
    

    while (ros::ok()){

        // Once drone has been armed for the first time reset the controller to remove accumulated errors
        if (!initial_arming && drone_armed){
            position_controller.reset();
            initial_arming = true;
        }


        geometry_msgs::Twist velocity_command;
        
        double dt = ros::Time::now().toSec() - last_loop_time;
        
        //should only run after first setpoint message and drone state received
        if (position_controller.reference_state_received() &&
            position_controller.state_received()){
            Eigen::Array4d control_outputs = position_controller.calculate_control_outputs(dt);

            velocity_command.linear.x += control_outputs[0]*dt;
            velocity_command.linear.y += control_outputs[1]*dt;
            velocity_command.linear.z += control_outputs[2]*dt;
            velocity_command.angular.z += control_outputs[3]*dt;
            
            velocity_command_pub.publish(velocity_command);

            last_loop_time = ros::Time::now().toSec();
        }
        

        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
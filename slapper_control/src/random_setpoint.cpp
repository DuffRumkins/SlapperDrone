#include <ros/ros.h>

//std
#include <stdlib.h> 

//tf2
#include <tf2/LinearMath/Quaternion.h>

//msg types
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>



mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

bool next_setpoint_flag = false;
void next_setpoint_flag_cb(const std_msgs::Bool::ConstPtr& flag_msg){
    next_setpoint_flag = flag_msg->data;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "track_cylinder_node");
    ros::NodeHandle nh;

    //subscribe to state to check for connection
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    //declare service clients for both the amring and setting autopilot mode 
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //TESTING SUBSCRIBER TO ALLOW CONTROL OF WHEN DRONE FOLLOWS NEW SETPOINT
    ros::Subscriber next_setpoint_flag_sub = nh.subscribe<std_msgs::Bool>
            ("next_setpoint", 1, next_setpoint_flag_cb);


    ros::Publisher local_setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/local_position_setpoint", 1);

    geometry_msgs::PoseStamped initial_setpoint;
    initial_setpoint.header.stamp = ros::Time::now();
    initial_setpoint.header.frame_id = "mocap_frame";   //Using mocap frame for later parity and since gazebo pose is being used to simulate mocap
    initial_setpoint.pose.position.z = 0.5;
    //Initial orientation can't be zeros since this will result in nan cmds from yaw controller
    initial_setpoint.pose.orientation.z = 0.001;
    initial_setpoint.pose.orientation.w = 1.0;
    


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //send a few setpoints before starting
    //necessary for switch to OFFBOARD mode

    for(int i = 100; ros::ok() && i > 0; --i){
        local_setpoint_pub.publish(initial_setpoint);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    geometry_msgs::PoseStamped setpoint = initial_setpoint;

    while(ros::ok()){
        //If current state is not OFFBOARD (and has been more than 5 seconds since last attempt to avoid flooding autopilot), try to switch mode to OFFBOARD
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        //If in OFFBOARD mode but drone is not yet armed (must have also been at least since 5 seconds since last attempt), try to arm drone
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        setpoint.header.stamp = ros::Time::now();

        if (next_setpoint_flag){

            setpoint.pose.position.x = 0.01 * (rand() % 400 + 1) - 2.0;
            setpoint.pose.position.y = 0.01 * (rand() % 400 + 1) - 2.0;
            setpoint.pose.position.z = 0.01 * (rand() % 200 + 1) + 0.5;
            
            tf2::Quaternion setpoint_orientation;
            float setpoint_yaw = M_PI*(rand() % 181)/181 - M_PI_2;
            setpoint_orientation.setRPY( 0, 0, setpoint_yaw);
            setpoint_orientation.normalize();

            setpoint.pose.orientation.z = setpoint_orientation.getZ();
            setpoint.pose.orientation.w = setpoint_orientation.getW();

            next_setpoint_flag = false;
        }

        local_setpoint_pub.publish(setpoint);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


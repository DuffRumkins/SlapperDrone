#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

//height_reached flag is true if drone above 0.95m
bool height_reached = false;
void position_cb(const geometry_msgs::PoseStamped::ConstPtr& pose){
    if (pose->pose.position.z > 0.95){
       height_reached = true; 
    }
    else
    {
        height_reached = false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    //subscribe to state to check for connection
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    //subscribe to local position to check for connection
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, position_cb);
    //publish to the local setpoint topic
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    //declare service clients for both the amring and setting autopilot mode 
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;
    // pose.pose.orientation.x = 0.0;
    // pose.pose.orientation.y = 0.0;
    // pose.pose.orientation.z = 0.05;
    // pose.pose.orientation.w = 0.999;

    //send a few setpoints before starting
    //necessary for switch to OFFBOARD mode
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    int count = 0;
    int yaw_angle = 0;
    tf2::Quaternion rotation_quarternion;

    while(ros::ok()){
        //If current state is not OFFBOARD (and has been more than 5 seconds since last attempt to avoid dlooding autopilot), try to switch mode to OFFBOARD
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

        if (height_reached){
            yaw_angle = count%360;
            rotation_quarternion.setRPY( 0, 0, yaw_angle * (M_PI/180.0));
            rotation_quarternion.normalize();

            pose.header.frame_id = "body_frame";

            pose.pose.position.x = 1;
            pose.pose.orientation.x = rotation_quarternion.getX();
            pose.pose.orientation.y = rotation_quarternion.getY();
            pose.pose.orientation.z = rotation_quarternion.getZ();
            pose.pose.orientation.w = rotation_quarternion.getW();

            count++;
        }

        //publish the local setpoint
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


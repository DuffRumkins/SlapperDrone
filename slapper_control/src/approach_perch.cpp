#include <ros/ros.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <slapper_vision/Perch.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>

geometry_msgs::PoseStamped camera_approach_setpoint_from_coefficients(slapper_vision::Perch coefficients){
    geometry_msgs::PoseStamped setpoint_camera;
    
    setpoint_camera.header.frame_id = "camera_link";
    setpoint_camera.pose.position.x = coefficients.axis_point.x;
    setpoint_camera.pose.position.y = coefficients.axis_point.y - 0.3; //-0.3 in y camera means a 30cm vertical offset above axis point
    setpoint_camera.pose.position.z = coefficients.axis_point.z;//0;
    //For the setpoint, we only care about the yaw of the perch (pitch in camera frame)
    double axis_pitch = -1.0 * atan2(coefficients.axis_direction.z,coefficients.axis_direction.x);
    tf2::Quaternion setpoint_axis_quaternion;
    setpoint_axis_quaternion.setRPY( 0, axis_pitch, 0);
    setpoint_axis_quaternion.normalize();
    setpoint_camera.pose.orientation.y = setpoint_axis_quaternion.getY();
    setpoint_camera.pose.orientation.w = setpoint_axis_quaternion.getW();



    return setpoint_camera;
}

geometry_msgs::PoseStamped transform_pose_camera_to_local(geometry_msgs::PoseStamped camera_pose, tf2_ros::Buffer& tfBuffer){
    geometry_msgs::PoseStamped local_pose;
    
    try{
        tfBuffer.transform(camera_pose, local_pose,"local_frame");
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    return local_pose;
}

visualization_msgs::Marker populate_local_approach_marker(geometry_msgs::PoseStamped approach_setpoint_local){
    visualization_msgs::Marker approach_point_marker;

    approach_point_marker.header.frame_id = "local_frame";
    approach_point_marker.header.stamp = ros::Time();
    approach_point_marker.type = visualization_msgs::Marker::SPHERE;
    approach_point_marker.action = visualization_msgs::Marker::ADD;

    //Approach point marker is in local_frame
    approach_point_marker.pose.position.x = approach_setpoint_local.pose.position.x;
    approach_point_marker.pose.position.y = approach_setpoint_local.pose.position.y;
    approach_point_marker.pose.position.z = approach_setpoint_local.pose.position.z;
    approach_point_marker.pose.orientation.x = 0.0;
    approach_point_marker.pose.orientation.y = 0.0;
    approach_point_marker.pose.orientation.z = 0.0;
    approach_point_marker.pose.orientation.w = 1.0;
    approach_point_marker.scale.x = 0.1;
    approach_point_marker.scale.y = 0.1;
    approach_point_marker.scale.z = 0.1;
    approach_point_marker.color.a = 1.0;
    approach_point_marker.color.r = 1.0;
    approach_point_marker.color.g = 0.0;
    approach_point_marker.color.b = 0.0;

    approach_point_marker.frame_locked = true;

    return approach_point_marker;
}

visualization_msgs::Marker populate_camera_approach_marker(geometry_msgs::PoseStamped approach_setpoint_camera){
    visualization_msgs::Marker approach_point_marker;

    approach_point_marker.header.frame_id = "camera_link";
    approach_point_marker.header.stamp = ros::Time();
    approach_point_marker.type = visualization_msgs::Marker::SPHERE;
    approach_point_marker.action = visualization_msgs::Marker::ADD;

    //Approach point marker is in camera_link frame
    approach_point_marker.pose.position.x = approach_setpoint_camera.pose.position.x;
    approach_point_marker.pose.position.y = approach_setpoint_camera.pose.position.y;
    approach_point_marker.pose.position.z = approach_setpoint_camera.pose.position.z;
    approach_point_marker.pose.orientation.x = 0.0;
    approach_point_marker.pose.orientation.y = 0.0;
    approach_point_marker.pose.orientation.z = 0.0;
    approach_point_marker.pose.orientation.w = 1.0;
    approach_point_marker.scale.x = 0.1;
    approach_point_marker.scale.y = 0.1;
    approach_point_marker.scale.z = 0.1;
    approach_point_marker.color.a = 1.0;
    approach_point_marker.color.r = 0.0;
    approach_point_marker.color.g = 0.0;
    approach_point_marker.color.b = 1.0;

    approach_point_marker.frame_locked = true;

    return approach_point_marker;
}


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

slapper_vision::Perch perch_coefficients;
void coefficients_cb(const slapper_vision::Perch::ConstPtr& perch_coefficients_msg)
{
    perch_coefficients = *perch_coefficients_msg;
}

bool approach_cylinder_flag = false;
void approach_cylinder_flag_cb(const std_msgs::Bool::ConstPtr& flag_msg){
    approach_cylinder_flag = flag_msg->data;
}

bool approach_point_being_validated_flag = false;
bool approach_point_chosen_flag = false;
void approach_point_chosen_flag_cb(const std_msgs::Bool::ConstPtr& flag_msg){
    approach_point_chosen_flag = flag_msg->data;
    approach_point_being_validated_flag = false;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "approach_perch_node");
    ros::NodeHandle nH;

    //subscribe to state to check for connection
    ros::Subscriber state_sub = nH.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    //declare service clients for both the amring and setting autopilot mode 
    ros::ServiceClient arming_client = nH.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nH.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //Subscriber for approach_cylinder_flag
    ros::Subscriber approach_cylinder_flag_sub = nH.subscribe<std_msgs::Bool>
            ("approach_cylinder", 1, approach_cylinder_flag_cb);
    //Subscriber for approach_point_chosen_flag
    ros::Subscriber approach_point_chosen_flag_sub = nH.subscribe<std_msgs::Bool>
            ("approach_point_chosen", 1, approach_point_chosen_flag_cb);
    //Subscriber for perch detections
    ros::Subscriber cylinder_coefficients_sub = nH.subscribe<slapper_vision::Perch>
            ("perch/cylinder_coefficients", 1, coefficients_cb);
    

    //Publisher for marker of approach point. Used for manual validation
    ros::Publisher local_approach_marker_pub = nH.advertise<visualization_msgs::Marker>("approach_marker",1);
    //Publisher for approach point in local frame. Once published, drone will fly here
    ros::Publisher local_setpoint_pub = nH.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",1);

    //Publisher for marker of approach point in camera frame.
    ros::Publisher camera_approach_marker_pub = nH.advertise<visualization_msgs::Marker>("camera_approach_marker",1);

    geometry_msgs::PoseStamped initial_setpoint;
    initial_setpoint.pose.position.x = -0.63;
    initial_setpoint.pose.position.y = 0.57;
    initial_setpoint.pose.position.z = 2.55;

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

    //Transform variables (must be persistent -> included in all scopes)
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::PoseStamped approach_setpoint_camera;
    geometry_msgs::PoseStamped approach_setpoint_local;

    visualization_msgs::Marker approach_point_marker_camera;
    visualization_msgs::Marker approach_point_marker_local;

    bool marker_published_flag = false;

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

        if (approach_cylinder_flag)
        {
            if (!approach_point_being_validated_flag && !approach_point_chosen_flag)
            {
                //if not approach point is currently being validated and no approach point has been chosen find new approach setpoint
                approach_setpoint_camera = camera_approach_setpoint_from_coefficients(perch_coefficients);
                approach_setpoint_local = transform_pose_camera_to_local(approach_setpoint_camera, tfBuffer);

                approach_point_being_validated_flag = true;
                marker_published_flag = false;
            }
            
            if (approach_point_being_validated_flag)
            {
                approach_point_marker_local = populate_local_approach_marker(approach_setpoint_local);
                approach_point_marker_camera = populate_camera_approach_marker(approach_setpoint_camera);
                local_setpoint_pub.publish(initial_setpoint);
                
                if (!marker_published_flag)
                {
                    local_approach_marker_pub.publish(approach_point_marker_local);
                    camera_approach_marker_pub.publish(approach_point_marker_camera);
                    marker_published_flag = true;
                }
            }
            else if (approach_point_chosen_flag)
            {
                ROS_INFO("Approaching point");
                local_setpoint_pub.publish(approach_setpoint_local);
            }
        }
        else{
            local_setpoint_pub.publish(initial_setpoint);
        }

        ros::spinOnce();
        rate.sleep();
    }
}
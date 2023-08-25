#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void transform_cb(const geometry_msgs::PoseStamped::ConstPtr& pose){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    double roll, pitch, yaw;
    tf2::Quaternion pose_q(pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z, pose->pose.orientation.w);
    tf2::Matrix3x3 pose_matrix(pose_q);
    pose_matrix.getRPY(roll, pitch, yaw);

    tf2::Quaternion frame_yaw_quaternion;
    frame_yaw_quaternion.setRPY( 0, 0, -yaw );
    frame_yaw_quaternion.normalize();

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "body_FLU"; 
    transformStamped.child_frame_id = "local_frame"; //Frame used when setting local setpoints
    //translation from origin body frame to origin local frame expressed in body frame 
    transformStamped.transform.translation.x = -1.0 * (pose->pose.position.x*cos(yaw) + pose->pose.position.y*sin(yaw));
    transformStamped.transform.translation.y = -1.0 * (-pose->pose.position.x*sin(yaw) + pose->pose.position.y*cos(yaw));;
    transformStamped.transform.translation.z = -1.0 * (pose->pose.position.z);        
    transformStamped.transform.rotation.x = frame_yaw_quaternion.getX();
    transformStamped.transform.rotation.y = frame_yaw_quaternion.getY();
    transformStamped.transform.rotation.z = frame_yaw_quaternion.getZ();
    transformStamped.transform.rotation.w = frame_yaw_quaternion.getW();

    br.sendTransform(transformStamped);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "body_frame_transform");
    ros::NodeHandle nh;

    //subscribe to state to check for connection
    ros::Subscriber state_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, transform_cb);

    ros::spin();
    return 0;
}
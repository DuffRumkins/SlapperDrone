#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class LocalSetpointSubscriber{
    public:
        std_msgs::Float64 msg_;
        float local_pose_x_, local_pose_y_, local_pose_z_;
        double local_pose_roll_, local_pose_pitch_, local_pose_yaw_;
        geometry_msgs::Twist velocity_command_;
        LocalSetpointSubscriber(std::string local_setpoint_topic, int queue_size)
        {
            localPoseSubscriber_ = nH_.subscribe("mavros/local_position/pose", 1, &LocalSetpointSubscriber::LocalPoseCallback, this);
            localSetpointSubscriber_ = nH_.subscribe(local_setpoint_topic ,queue_size, &LocalSetpointSubscriber::LocalSetpointCallback, this);
        }
        void LocalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr pose_msg)
        {
            local_pose_x_ = pose_msg->pose.position.x;
            local_pose_y_ = pose_msg->pose.position.y;
            local_pose_z_ = pose_msg->pose.position.z;

            tf2::Quaternion local_pose_orientation(pose_msg->pose.orientation.x,
                                                   pose_msg->pose.orientation.y,
                                                   pose_msg->pose.orientation.z,
                                                   pose_msg->pose.orientation.w);
            tf2::Matrix3x3 local_pose_matrix(local_pose_orientation);
            local_pose_matrix.getRPY(local_pose_roll_, local_pose_pitch_, local_pose_yaw_);
        }
        void LocalSetpointCallback(const geometry_msgs::PoseStamped::ConstPtr local_setpoint_msg)
        {
            msg_.data = local_pose_x_;
            localPosePublisher_x.publish(msg_);
            msg_.data = local_setpoint_msg->pose.position.x;
            localSetpointPublisher_x.publish(msg_);

            msg_.data = local_pose_y_;
            localPosePublisher_y.publish(msg_);
            msg_.data = local_setpoint_msg->pose.position.y;
            localSetpointPublisher_y.publish(msg_);

            msg_.data = local_pose_z_;
            localPosePublisher_z.publish(msg_);
            msg_.data = local_setpoint_msg->pose.position.z;
            localSetpointPublisher_z.publish(msg_);

            double roll_local_setpoint, pitch_local_setpoint, yaw_local_setpoint;
            tf2::Quaternion local_setpoint_orientation(local_setpoint_msg->pose.orientation.x,
                                                       local_setpoint_msg->pose.orientation.y,
                                                       local_setpoint_msg->pose.orientation.z,
                                                       local_setpoint_msg->pose.orientation.w);
            tf2::Matrix3x3 local_setpoint_matrix(local_setpoint_orientation);
            local_setpoint_matrix.getRPY(roll_local_setpoint, pitch_local_setpoint, yaw_local_setpoint);

            msg_.data = local_pose_yaw_;
            localPosePublisher_yaw.publish(msg_);
            msg_.data = yaw_local_setpoint;
            localSetpointPublisher_yaw.publish(msg_);
        }
        void localVelocityCommandSubscriber_xCallback(const std_msgs::Float64::ConstPtr x_rate_cmd_msg){
            velocity_command_.linear.x = x_rate_cmd_msg->data;
        }
        void localVelocityCommandSubscriber_yCallback(const std_msgs::Float64::ConstPtr y_rate_cmd_msg){
            velocity_command_.linear.y= y_rate_cmd_msg->data;
        }
        void localVelocityCommandSubscriber_zCallback(const std_msgs::Float64::ConstPtr z_rate_cmd_msg){
            velocity_command_.linear.z = z_rate_cmd_msg->data;
        }
        void localVelocityCommandSubscriber_yawCallback(const std_msgs::Float64::ConstPtr yaw_rate_cmd_msg){
            velocity_command_.angular.z = yaw_rate_cmd_msg->data;
        }
    protected:
        ros::NodeHandle nH_;
        ros::Subscriber localPoseSubscriber_;
        ros::Subscriber localSetpointSubscriber_;

        ros::Publisher localPosePublisher_x = nH_.advertise<std_msgs::Float64>("/drone_local_position/x",1);
        ros::Publisher localPosePublisher_y = nH_.advertise<std_msgs::Float64>("/drone_local_position/y",1);
        ros::Publisher localPosePublisher_z = nH_.advertise<std_msgs::Float64>("/drone_local_position/z",1);
        ros::Publisher localPosePublisher_yaw = nH_.advertise<std_msgs::Float64>("/drone_local_position/yaw",1);

        ros::Publisher localSetpointPublisher_x = nH_.advertise<std_msgs::Float64>("/x_local_ref",1);
        ros::Publisher localSetpointPublisher_y = nH_.advertise<std_msgs::Float64>("/y_local_ref",1);
        ros::Publisher localSetpointPublisher_z = nH_.advertise<std_msgs::Float64>("/z_local_ref",1);
        ros::Publisher localSetpointPublisher_yaw = nH_.advertise<std_msgs::Float64>("/yaw_local_ref",1);

        ros::Subscriber localVelocityCommandSubscriber_x = nH_.subscribe("/x_rate_cmd", 1,
                                                                             &LocalSetpointSubscriber::localVelocityCommandSubscriber_xCallback, this);
        ros::Subscriber localVelocityCommandSubscriber_y = nH_.subscribe("/y_rate_cmd", 1,
                                                                             &LocalSetpointSubscriber::localVelocityCommandSubscriber_yCallback, this);
        ros::Subscriber localVelocityCommandSubscriber_z = nH_.subscribe("/z_rate_cmd", 1,
                                                                             &LocalSetpointSubscriber::localVelocityCommandSubscriber_zCallback, this);
        ros::Subscriber localVelocityCommandSubscriber_yaw = nH_.subscribe("/yaw_rate_cmd", 1,
                                                                             &LocalSetpointSubscriber::localVelocityCommandSubscriber_yawCallback, this);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_position_controller_node");
    ros::NodeHandle nh;
            
    //publish to the local setpoint velocity topic
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 1);


    LocalSetpointSubscriber local_setpoint_subscriber("/local_position_setpoint", 1);

    while (ros::ok){
        velocity_pub.publish(local_setpoint_subscriber.velocity_command_);

        ros::spinOnce();
    }

    return 0;
}


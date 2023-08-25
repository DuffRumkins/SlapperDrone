#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

class OdomToPoseRemapper{
    public:
        OdomToPoseRemapper(std::string original_odom_topic, int original_odom__queue, 
                    std::string pose_topic, int pose_queue)
        {
            originalOdomSubscriber_ = nH_.subscribe<nav_msgs::Odometry>
                                    (original_odom_topic, original_odom__queue, &OdomToPoseRemapper::OdomCallback, this);
            
            posePublisher_ = nH_.advertise<geometry_msgs::PoseStamped>(pose_topic, pose_queue);
        }

        void OdomCallback(const nav_msgs::Odometry::ConstPtr& original_odom)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.pose = original_odom->pose.pose;
            posePublisher_.publish(pose);
        }
    protected:
        ros::NodeHandle nH_;
        ros::Subscriber originalOdomSubscriber_;
        ros::Publisher posePublisher_;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazeboOdomRemapper");

    OdomToPoseRemapper odometry_to_pose_remapper("/gazebo_mocap",10,"/mavros/vision_pose/pose",10);

    ros::spin();

    return 0;
}
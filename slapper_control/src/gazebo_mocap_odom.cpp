#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>

class GazeboMocapOdom{
    public:
        GazeboMocapOdom(std::string gazebo_topic, int gazebo_queue, 
                    std::string odom_topic, int odom_queue)
        {
            gazeboSubscriber_ = nH_.subscribe<gazebo_msgs::ModelStates>
                                    (gazebo_topic, gazebo_queue, &GazeboMocapOdom::GazeboCallback, this);
            
            odomPublisher_ = nH_.advertise<nav_msgs::Odometry>(odom_topic, odom_queue);
        }

        void GazeboCallback(const gazebo_msgs::ModelStates::ConstPtr& gazebo_models)
        {
            nav_msgs::Odometry odom;

            odom.header.stamp = ros::Time::now();
            odom.header.frame_id = "mocap_frame";

            odom.pose.pose = gazebo_models->pose[2];
            odom.pose.covariance[0] = 0.0001;
            odom.pose.covariance[7] = 0.0001;
            odom.pose.covariance[14] = 0.0001;
            odom.pose.covariance[21] = 0.0001;
            odom.pose.covariance[28] = 0.0001;
            odom.pose.covariance[35] = 0.0001;

            odom.twist.twist = gazebo_models->twist[2];
            odom.twist.covariance[0] = 0.0001;
            odom.twist.covariance[7] = 0.0001;
            odom.twist.covariance[14] = 0.0001;
            odom.twist.covariance[21] = 0.0001;
            odom.twist.covariance[28] = 0.0001;
            odom.twist.covariance[35] = 0.0001;

            odomPublisher_.publish(odom);
        }
    protected:
        ros::NodeHandle nH_;
        ros::Subscriber gazeboSubscriber_;
        ros::Publisher odomPublisher_;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazeboMocapOdom");

    GazeboMocapOdom gazebo_mocap_odom("/gazebo/model_states",10,"/gazebo_mocap",10);

    ros::spin();

    return 0;
}
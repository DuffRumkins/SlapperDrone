#include <ros/ros.h>

//tf2 + tf2_ros
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>

//Eigen
#include <Eigen/Dense>

//pcl
#include <pcl/common/distances.h>
#include <pcl/common/impl/angles.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/crop_box.h>

//msg types
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <slapper_vision/Perch.h>
#include <visualization_msgs/Marker.h>

#include <slapper_control/DoublePublisherSingleSubscriber.hpp>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointNormal PointN;
typedef pcl::PointCloud<PointT> CloudRGB;

//Transform variables (must be persistent -> included in all scopes)
tf2_ros::Buffer tfBuffer;



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

geometry_msgs::PoseStamped transform_pose_local_to_camera(geometry_msgs::PoseStamped local_pose, tf2_ros::Buffer& tfBuffer){
    geometry_msgs::PoseStamped camera_pose;
    
    try{
        tfBuffer.transform(local_pose, camera_pose,"camera_link");
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    return camera_pose;
}

void rotate_vector_by_quaternion(Eigen::Vector3f& v, tf2::Quaternion q){
    // Extract the vector part of the quaternion
    Eigen::Vector3f u(q.getX(), q.getY(), q.getZ());

    // Extract the scalar part of the quaternion
    float s = q.getW();

    // Do the math
    v = 2.0 * u.dot(v) * u + (s*s - u.dot(u)) * v + 2.0 * s * u.cross(v);
}

//Function to convert from pose (WITH QUARTERNION AXIS DIRECTION AS ORIENTATION) to cylinder model perch_coefficients
void pose_to_perch_coefficients(geometry_msgs::PoseStamped pose, float radius, float fitting_score, slapper_vision::Perch& perch_coefficients){
    //convert pose quaternion to vector
    Eigen::Vector3f axis_vector_local(1.0,0,0);
    tf2::Quaternion axis_q_local(pose.pose.orientation.x,
                                 pose.pose.orientation.y,
                                 pose.pose.orientation.z,
                                 pose.pose.orientation.w);
    rotate_vector_by_quaternion(axis_vector_local,axis_q_local);

    //To allow us to take the average of the vectors later, all vectors need to use the same conventions
    //It was chosen to use normalised vectors where the x entry is always positive
    axis_vector_local /= axis_vector_local.norm();
    if (axis_vector_local[0] < 0){
        axis_vector_local *= -1;
    }
    // std::cerr << "new axis_vector_local is: " << axis_vector_local << " \n-----------------------------------";

    perch_coefficients.axis_point.x = pose.pose.position.x;
    perch_coefficients.axis_point.y = pose.pose.position.y;
    perch_coefficients.axis_point.z = pose.pose.position.z;
    perch_coefficients.axis_direction.x = axis_vector_local[0];
    perch_coefficients.axis_direction.y = axis_vector_local[1];
    perch_coefficients.axis_direction.z = axis_vector_local[2];

    perch_coefficients.radius = radius;
    perch_coefficients.fitting_score = fitting_score;
}

bool freeze_flag = false;
void freeze_perch_flag_cb(const std_msgs::Bool::ConstPtr& freeze_flag_msg){
    freeze_flag = freeze_flag_msg->data;
}

bool first_detection_flag = false;
slapper_vision::Perch perch_coefficients_camera;
slapper_vision::Perch perch_coefficients_local;
geometry_msgs::PoseStamped perch_pose_local;
void coefficients_cb(const slapper_vision::Perch::ConstPtr& perch_coefficients_msg)
{
    if (!freeze_flag){
        perch_coefficients_camera = *perch_coefficients_msg;

        float perch_radius = perch_coefficients_camera.radius;
        float fitting_score = perch_coefficients_camera.fitting_score;

        //Convert axis vector to quarternion format
        double axis_roll = atan2(perch_coefficients_camera.axis_direction.z,perch_coefficients_camera.axis_direction.y);
        double axis_pitch = -1.0 * atan2(perch_coefficients_camera.axis_direction.z,perch_coefficients_camera.axis_direction.x);
        double axis_yaw = atan2(perch_coefficients_camera.axis_direction.y,perch_coefficients_camera.axis_direction.x);
        tf2::Quaternion axis_quaternion;
        axis_quaternion.setRPY( axis_roll, axis_pitch, axis_yaw);
        axis_quaternion.normalize();

        geometry_msgs::PoseStamped perch_pose_camera;
        perch_pose_camera.header.frame_id = "camera_link";
        perch_pose_camera.pose.position.x = perch_coefficients_camera.axis_point.x;
        perch_pose_camera.pose.position.y = perch_coefficients_camera.axis_point.y;
        perch_pose_camera.pose.position.z = perch_coefficients_camera.axis_point.z;
        perch_pose_camera.pose.orientation.x = axis_quaternion.getX();
        perch_pose_camera.pose.orientation.y = axis_quaternion.getY();
        perch_pose_camera.pose.orientation.z = axis_quaternion.getZ();
        perch_pose_camera.pose.orientation.w = axis_quaternion.getW();

        perch_pose_local = transform_pose_camera_to_local(perch_pose_camera, tfBuffer);
        pose_to_perch_coefficients(perch_pose_local, perch_radius, fitting_score, perch_coefficients_local);

        first_detection_flag = true;
    }
}

//Create marker for crop_box cloud
visualization_msgs::Marker createCropBoxMarker(Eigen::Vector4f min_pt, Eigen::Vector4f max_pt, 
                                                Eigen::Vector3f translation, Eigen::Vector3f rotation,
                                                bool clear_path){
  visualization_msgs::Marker cropbox_marker;
  cropbox_marker.header.frame_id = "camera_link";
  cropbox_marker.header.stamp = ros::Time();
  cropbox_marker.ns = "cropbox";
  cropbox_marker.id = 0;
  cropbox_marker.type = visualization_msgs::Marker::CUBE;
  cropbox_marker.action = visualization_msgs::Marker::ADD;

  cropbox_marker.pose.position.x = translation[0];
  cropbox_marker.pose.position.y = translation[1];
  cropbox_marker.pose.position.z = translation[2];
  
  tf2::Quaternion orientation_quarternion;
  orientation_quarternion.setRPY(rotation[0], rotation[1], rotation[2]);
  orientation_quarternion.normalize();

  cropbox_marker.pose.orientation.x = orientation_quarternion.getX();
  cropbox_marker.pose.orientation.y = orientation_quarternion.getY();
  cropbox_marker.pose.orientation.z = orientation_quarternion.getZ();
  cropbox_marker.pose.orientation.w = orientation_quarternion.getW();
  cropbox_marker.scale.x = std::abs(max_pt[0] - min_pt[0]);
  cropbox_marker.scale.y = std::abs(max_pt[1] - min_pt[1]);
  cropbox_marker.scale.z = std::abs(max_pt[2] - min_pt[2]);
  cropbox_marker.color.a = 0.3; // Don't forget to set the alpha!

  cropbox_marker.frame_locked = true;

  if (clear_path){
    cropbox_marker.color.r = 0.0;
    cropbox_marker.color.g = 1.0;
  }
  else{
    cropbox_marker.color.r = 1.0;
    cropbox_marker.color.g = 0.0;
  }
  cropbox_marker.color.b = 0.0;

  return cropbox_marker;
}



mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

double pos_x_local, pos_y_local, pos_z_local;
double roll, pitch, yaw;
void position_cb(const geometry_msgs::PoseStamped::ConstPtr& pose){
    pos_x_local = pose->pose.position.x;
    pos_y_local = pose->pose.position.y;
    pos_z_local = pose->pose.position.z;

    tf2::Quaternion pose_q(pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z, pose->pose.orientation.w);
    tf2::Matrix3x3 pose_matrix(pose_q);
    pose_matrix.getRPY(roll, pitch, yaw);
}

template<>
void DoublePublisherSingleSubscriber<CloudRGB, visualization_msgs::Marker, CloudRGB>::subscriberCallback(const CloudRGB::ConstPtr& cloud)
{
    //Run for every cloud after first detection
    if (first_detection_flag)
    {
        if (freeze_flag){
            //Delay of 0.5 seconds to allow transforms to catch up.
            perch_pose_local.header.stamp = ros::Time::now()- ros::Duration(0.5);
        }

        geometry_msgs::PoseStamped perch_pose_camera = transform_pose_local_to_camera(perch_pose_local, tfBuffer);
        std::cerr << "perch_pose_camera: " << perch_pose_camera << "\n";

        double perch_x = perch_pose_camera.pose.position.x;
        double perch_y = perch_pose_camera.pose.position.y - 0.3;
        double perch_z = perch_pose_camera.pose.position.z;

        float box_x_length = std::abs(perch_x);
        float box_y_length = std::abs(perch_y);
        float box_z_length = std::abs(perch_z);

        pcl::CropBox<PointT> cropBoxFilter (true);
        cropBoxFilter.setInputCloud(cloud);
        Eigen::Vector4f min_pt (-0.5*box_x_length, -0.5*box_y_length, -0.5*box_z_length, 1.0f);
        Eigen::Vector4f max_pt (0.5*box_x_length, 0.5*box_y_length, 0.5*box_z_length, 1.0f);

        Eigen::Vector3f translation(0.5*box_x_length, 
                                    0.5*box_y_length, 
                                    0.5*box_z_length);

        //Do not need a rotation since the camera frame is already rotated!
        Eigen::Vector3f rotation(0.0, 0.0, 0.0);

        // Cropbox slighlty bigger then bounding box of points
        cropBoxFilter.setMin (min_pt);
        cropBoxFilter.setMax (max_pt);

        // std::cerr << "min_pt: " << min_pt << "\n";
        // std::cerr << "max_pt: " << max_pt << "\n";
        // std::cerr << "translation: " << translation << "\n";

        cropBoxFilter.setTranslation(translation);

        cropBoxFilter.setRotation(rotation);

        // Create cloud for filtered points to be saved in
        pcl::PointCloud<PointT> cloud_box;
        cropBoxFilter.filter (cloud_box);

        bool clear_path = false;
        if (cloud_box.size()<=0){
            clear_path = true;
        }

        publisherObject.publish(cloud_box);

        visualization_msgs::Marker cropbox_marker =  createCropBoxMarker(min_pt, max_pt, translation, rotation, clear_path);

        secondPublisherObject.publish(cropbox_marker);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning_box_node");
    ros::NodeHandle nh;
    tf2_ros::TransformListener tfListener(tfBuffer);

    //subscribe to state to check for connection
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    //declare service clients for both the amring and setting autopilot mode 
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //subscribe to local position to determine Euler angles between world and body FLU frames
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 1, position_cb);

    ros::Subscriber cylinder_coefficients_sub = nh.subscribe<slapper_vision::Perch>
            ("perch/cylinder_coefficients", 1, coefficients_cb);

    ros::Subscriber freeze_perch_flag_sub = nh.subscribe<std_msgs::Bool>
            ("freeze_perch", 1, freeze_perch_flag_cb);

    DoublePublisherSingleSubscriber<CloudRGB, visualization_msgs::Marker, CloudRGB> boxDrawer("output_cloud", "cropbox_marker", "/iris_depth_camera/camera/depth/points",1);


    ros::Publisher local_setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 1);

    geometry_msgs::PoseStamped initial_setpoint;
    initial_setpoint.pose.position.x = 3.05;//3.23;
    initial_setpoint.pose.position.y = -1.0;//;-0.8;
    initial_setpoint.pose.position.z = 2.5;//2.3;
    initial_setpoint.pose.orientation.z = 0.819;//0.707;
    initial_setpoint.pose.orientation.w = 0.574;//0.707;

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

        local_setpoint_pub.publish(initial_setpoint);
        

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


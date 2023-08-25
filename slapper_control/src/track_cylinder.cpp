#include <ros/ros.h>

//tf2 + tf2_ros
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>

//Eigen
#include <Eigen/Dense>

//pcl
#include <pcl/common/distances.h>
#include <pcl/common/impl/angles.hpp>

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


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

bool track_cylinder_flag = false;
void track_cylinder_flag_cb(const std_msgs::Bool::ConstPtr& flag_msg){
    track_cylinder_flag = flag_msg->data;
}

bool next_setpoint_flag = false;
void next_setpoint_flag_cb(const std_msgs::Bool::ConstPtr& flag_msg){
    next_setpoint_flag = flag_msg->data;
}

slapper_vision::Perch perch_coefficients;
bool new_detection = false;
void coefficients_cb(const slapper_vision::Perch::ConstPtr& perch_coefficients_msg)
{
    perch_coefficients = *perch_coefficients_msg;
    new_detection = true;
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

    perch_coefficients.axis_point.x = pose.pose.position.x;
    perch_coefficients.axis_point.y = pose.pose.position.y;
    perch_coefficients.axis_point.z = pose.pose.position.z;
    perch_coefficients.axis_direction.x = axis_vector_local[0];
    perch_coefficients.axis_direction.y = axis_vector_local[1];
    perch_coefficients.axis_direction.z = axis_vector_local[2];

    perch_coefficients.radius = radius;
    perch_coefficients.fitting_score = fitting_score;
}

float find_acute_angle_vectors(Eigen::Vector4f v1, Eigen::Vector4f v2){
    float angle = acos(v1.dot(v2)/(v1.norm()*v2.norm()));

    //we are interested in the acute angle between the two lines
    if (angle > M_PI_2){
        return M_PI - angle;
    }
    return angle;
}


bool check_cylinder(slapper_vision::Perch new_perch, slapper_vision::Perch reference_perch, float angle_limit, float distance_limit){
    //calculate angle between cylinder axes
    Eigen::Vector4f new_axis_dir(new_perch.axis_direction.x, new_perch.axis_direction.y, new_perch.axis_direction.z, 0);
    Eigen::Vector4f reference_axis_dir(reference_perch.axis_direction.x, reference_perch.axis_direction.y, reference_perch.axis_direction.z, 0);
    
    float axis_angle = pcl::rad2deg(find_acute_angle_vectors(new_axis_dir,reference_axis_dir));

    //calculate distance between point on new axis and the reference axis
    Eigen::Vector4f new_axis_point(new_perch.axis_point.x, new_perch.axis_point.y, new_perch.axis_point.z, 0);
    Eigen::Vector4f reference_axis_point(reference_perch.axis_point.x, reference_perch.axis_point.y, reference_perch.axis_point.z, 0);

    double point_axis_distance = sqrt(pcl::sqrPointToLineDistance(new_axis_point, reference_axis_point,reference_axis_dir));

    if (axis_angle <= angle_limit && point_axis_distance <= distance_limit){
        return true;
    }

    return false;
}

bool check_drone_on_axis(slapper_vision::Perch new_perch, float drone_distance_limit){
    //check to see if drone already on cylinder axis
    Eigen::Vector4f drone_pos_local(pos_x_local, pos_y_local, pos_z_local, 0);
    Eigen::Vector4f new_axis_dir(new_perch.axis_direction.x, new_perch.axis_direction.y, new_perch.axis_direction.z, 0);
    Eigen::Vector4f new_axis_point(new_perch.axis_point.x, new_perch.axis_point.y, new_perch.axis_point.z, 0);

    if (sqrt(pcl::sqrPointToLineDistance(drone_pos_local, new_axis_point,new_axis_dir)) <= drone_distance_limit){
        return true;
    }

    return false;
}

float update_weighted_average(float prev_weighted_avg, float sum_prev_weights, float new_data, float new_weight){
    return (prev_weighted_avg*sum_prev_weights + new_data*new_weight)/(sum_prev_weights + new_weight);
}

void update_reference_perch(slapper_vision::Perch& reference_perch, slapper_vision::Perch new_perch, float reference_weight_limit){
    float sum_previous_weights = reference_perch.fitting_score;
    float new_weight = new_perch.fitting_score;

    reference_perch.axis_point.x = update_weighted_average(reference_perch.axis_point.x, sum_previous_weights,
                                                           new_perch.axis_point.x, new_weight);
    reference_perch.axis_point.y = update_weighted_average(reference_perch.axis_point.y, sum_previous_weights,
                                                           new_perch.axis_point.y, new_weight);
    reference_perch.axis_point.z = update_weighted_average(reference_perch.axis_point.z, sum_previous_weights,
                                                           new_perch.axis_point.z, new_weight);

    float updated_ref_axis_x = update_weighted_average(reference_perch.axis_direction.x, sum_previous_weights,
                                                               new_perch.axis_direction.x, new_weight);
    float updated_ref_axis_y = update_weighted_average(reference_perch.axis_direction.y, sum_previous_weights,
                                                               new_perch.axis_direction.y, new_weight);
    float updated_ref_axis_z = update_weighted_average(reference_perch.axis_direction.z, sum_previous_weights,
                                                               new_perch.axis_direction.z, new_weight);
    Eigen::Vector3f reference_axis_direction(updated_ref_axis_x, updated_ref_axis_y, updated_ref_axis_z);
    reference_axis_direction /= reference_axis_direction.norm();

    reference_perch.axis_direction.x = reference_axis_direction[0];
    reference_perch.axis_direction.y = reference_axis_direction[1];
    reference_perch.axis_direction.z = reference_axis_direction[2];

    reference_perch.radius = update_weighted_average(reference_perch.radius, sum_previous_weights,
                                                     new_perch.radius, new_weight);

    if (reference_perch.fitting_score + new_weight > reference_weight_limit){
        reference_perch.fitting_score = reference_weight_limit;
    }
    else{
        reference_perch.fitting_score = sum_previous_weights + new_weight;
    }    
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

    //subscribe to local position to determine Euler angles between world and body FLU frames
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 1, position_cb);

    ros::Subscriber track_cylinder_flag_sub = nh.subscribe<std_msgs::Bool>
            ("track_cylinder", 1, track_cylinder_flag_cb);

    //TESTING SUBSCRIBER TO ALLOW CONTROL OF WHEN DRONE FOLLOWS NEW SETPOINT
    ros::Subscriber next_setpoint_flag_sub = nh.subscribe<std_msgs::Bool>
            ("next_setpoint", 1, next_setpoint_flag_cb);

    ros::Subscriber cylinder_coefficients_sub = nh.subscribe<slapper_vision::Perch>
            ("perch/cylinder_coefficients", 1, coefficients_cb);

    //local position setpoint publisher to get drone to starting position
    // ros::Publisher local_setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>
    //         ("mavros/setpoint_position/local", 10);
    
    // Local position setpoint publisher to PX4 position controller
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

    //Transform variables (must be persistent -> included in all scopes)
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    float perch_axis_angle_limit = 20; //[deg] Maximum acceptable angle between new perch and the perch reference
    float perch_distance_axis_limit = 0.3; //[m] Maximum acceptable distance between new perch axis point and the perch reference axis vector
    float drone_perch_axis_distance_limit = 0.05; //[m] Maximum distance between drone and new perch axis for drone to be considered on the axis
    float reference_weight_limit = 5;   //Maximum weight that the reference perch can have (to avoid new detections being ignored)
    int initial_reference_checks = 10; //Number of detections used to choose best initial perch

    slapper_vision::Perch potential_initial_reference_perch_coefficients;
    int initial_reference_counter = 0;

    bool initial_reference_set = false;

    slapper_vision::Perch reference_perch_coefficients;

    geometry_msgs::PoseStamped previous_setpoint_local = initial_setpoint;

    //TESTING VARIABLE TO LIMIT TO A SINGLE SETPOINT
    bool perch_setpoint_published = false;

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

        geometry_msgs::PoseStamped setpoint_local;

        //If track_cylinder topic is posting true then track the cylinder (perch)
        if (track_cylinder_flag){                     
            if (new_detection && next_setpoint_flag)// && !perch_setpoint_published) //!perch_setpoint published is to ensure that only the first perch detection setpoint is used
            {
                //--------------------------------------TEST-----------------------------------------------
                float perch_radius = perch_coefficients.radius;
                float fitting_score = perch_coefficients.fitting_score;
               
                //Convert axis vector to quarternion format
                double axis_roll = atan2(perch_coefficients.axis_direction.z,perch_coefficients.axis_direction.y);
                double axis_pitch = -1.0 * atan2(perch_coefficients.axis_direction.z,perch_coefficients.axis_direction.x);
                double axis_yaw = atan2(perch_coefficients.axis_direction.y,perch_coefficients.axis_direction.x);
                tf2::Quaternion axis_quaternion;
                axis_quaternion.setRPY( axis_roll, axis_pitch, axis_yaw);
                axis_quaternion.normalize();

                geometry_msgs::PoseStamped perch_pose_camera;
                perch_pose_camera.header.frame_id = "camera_link";
                perch_pose_camera.pose.position.x = perch_coefficients.axis_point.x;
                perch_pose_camera.pose.position.y = perch_coefficients.axis_point.y;
                perch_pose_camera.pose.position.z = perch_coefficients.axis_point.z;
                perch_pose_camera.pose.orientation.x = axis_quaternion.getX();
                perch_pose_camera.pose.orientation.y = axis_quaternion.getY();
                perch_pose_camera.pose.orientation.z = axis_quaternion.getZ();
                perch_pose_camera.pose.orientation.w = axis_quaternion.getW();

                geometry_msgs::PoseStamped setpoint_camera;
                setpoint_camera.header.frame_id = "camera_link";
                setpoint_camera.pose.position.x = perch_coefficients.axis_point.x;
                setpoint_camera.pose.position.y = perch_coefficients.axis_point.y - 0.3; //-0.3 in y camera means a 30cm vertical offset above point
                setpoint_camera.pose.position.z = perch_coefficients.axis_point.z;//0;
                //For the setpoint, we only care about the yaw of the perch (pitch in camera frame)
                tf2::Quaternion setpoint_axis_quaternion;
                setpoint_axis_quaternion.setRPY( 0, axis_pitch, 0);
                setpoint_axis_quaternion.normalize();
                setpoint_camera.pose.orientation.y = setpoint_axis_quaternion.getY();
                setpoint_camera.pose.orientation.w = setpoint_axis_quaternion.getW();
                
                
                geometry_msgs::PoseStamped perch_pose_local = transform_pose_camera_to_local(perch_pose_camera, tfBuffer);
                
                slapper_vision::Perch new_perch_coefficients;
                pose_to_perch_coefficients(perch_pose_local, perch_radius, fitting_score, new_perch_coefficients);
                
                setpoint_local = transform_pose_camera_to_local(setpoint_camera, tfBuffer);

                //initial reference is chosen as highest scoring detection of set number of initial detections
                if (!initial_reference_set){

                    if (new_perch_coefficients.fitting_score>potential_initial_reference_perch_coefficients.fitting_score){
                        potential_initial_reference_perch_coefficients = new_perch_coefficients;
                    }
                    initial_reference_counter++;
                    
                    if (initial_reference_counter >= initial_reference_checks){
                        reference_perch_coefficients = potential_initial_reference_perch_coefficients;
                        //Set fitting score to 0 so the initial detection is not weighted twice during reference perch update
                        reference_perch_coefficients.fitting_score = 0;
                        initial_reference_set = true;
                    }
                }

                //if cylinder passes check publish new setpoint
                if (check_cylinder(new_perch_coefficients,reference_perch_coefficients,perch_axis_angle_limit,perch_distance_axis_limit)){
                    //add to reference average mulitplied by score
                    update_reference_perch(reference_perch_coefficients, new_perch_coefficients, reference_weight_limit);

                    if(check_drone_on_axis(new_perch_coefficients,drone_perch_axis_distance_limit)){
                        //If drone is on x axis we maintain position in x-y plane
                        setpoint_local.pose.position.x = pos_x_local;
                        setpoint_local.pose.position.y = pos_y_local;
                    }

                    //publish local setpoint
                    local_setpoint_pub.publish(setpoint_local);

                    previous_setpoint_local = setpoint_local;

                    new_detection = false;
                    next_setpoint_flag = false;

                    //perch_setpoint_published = true;
                }
                //else re-publish previous setpoint
                else{
                    local_setpoint_pub.publish(previous_setpoint_local);
                }
               
            }


            else{
                // ROS_INFO("next_setpoint_flag = %d && new_detection = %d", next_setpoint_flag, new_detection);
                local_setpoint_pub.publish(previous_setpoint_local);
            }
            
        }
        //Otherwise fly to initial (local) position setpoint
        else{
            local_setpoint_pub.publish(initial_setpoint);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


#include <ros/ros.h>
#include <slapper_vision/DoublePublisherSingleSubscriber.hpp>
#include <cmath>
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>
#include <slapper_vision/cylinder_detectorConfig.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <slapper_vision/Perch.h>

//PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/common/distances.h>
#include <pcl/common/centroid.h>
#include <pcl/common/impl/angles.hpp>

//OpenCV
#include <opencv2/opencv.hpp>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointNormal PointN;
typedef pcl::PointCloud<PointT> CloudRGB;

//Define global variable
double LAB_threshold;
double vg_leaf;
double pt_min;
double pt_max;
double cluster_tolerance;
double cluster_size_perc_min;
double cluster_size_perc_max;
int normal_k;
double cylinder_normal_distance;
int cylinder_max_iterations;
double cylinder_inlier_distance;
double cylinder_radius_min;
double cylinder_radius_max;
double cylinder_scoring_distance;
bool optimise_cylinder;
double cylinder_min_roll;
double cylinder_max_roll;
double cylinder_min_pitch;
double cylinder_max_pitch;
double cylinder_min_yaw;
double cylinder_max_yaw;

//Static parameters
bool cluster_testing;
bool cylinder_testing;
bool centroid_testing;
bool coefficient_testing;

//Set defaults to cluster_testing (arbitrarily) -> variables have to be initially declared in same scope as publisher/subscriber object
std::string first_publish_topic = "perch/cluster";
std::string second_publish_topic = "perch/cylinder";
std::string subscribe_topic = "/iris_depth_camera/camera/depth/points";

std::string publish_frame = "camera_link";

//-----------------FUNCTION DEFINITIONS-----------------------------------

bool
enforceLABDistanceSimilarity(const PointT& point_a, const PointT& point_b, float squared_distance)
{
  cv::Mat3f rgb_a (cv::Vec3f(point_a.r/255.0, point_a.g/255.0, point_a.b/255.0));
  cv::Mat3f rgb_b (cv::Vec3f(point_b.r/255.0, point_b.g/255.0, point_b.b/255.0));
  cv::Mat3f lab_a;
  cv::Mat3f lab_b;

  cv::cvtColor(rgb_a,lab_a,cv::COLOR_RGB2Lab);
  cv::cvtColor(rgb_b,lab_b,cv::COLOR_RGB2Lab);

  cv::Vec3f lab_a_vec = lab_a[0][0];
  cv::Vec3f lab_b_vec = lab_b[0][0];

  if (std::sqrt(pow((lab_a_vec[0] - lab_b_vec[0]),2)+pow((lab_a_vec[1] - lab_b_vec[1]),2) + pow((lab_a_vec[2] - lab_b_vec[2]),2)) < LAB_threshold)
  {
    return (true);
  }
  return (false);
}

//Function for downsampling point cloud 
CloudRGB::Ptr cloudDownsampling(const CloudRGB::ConstPtr& cloud){
  pcl::VoxelGrid<PointT> vg;
  CloudRGB::Ptr cloud_downsampled (new CloudRGB);
  vg.setInputCloud (cloud);
  vg.setLeafSize (vg_leaf, vg_leaf, vg_leaf);
  vg.filter(*cloud_downsampled);
  
  return cloud_downsampled;
}

//Function for applying passthrough filter to point cloud
CloudRGB::Ptr cloudPassThrough(CloudRGB::Ptr& cloud){
  pcl::PassThrough<PointT> filter;
  CloudRGB::Ptr cloud_cropped (new CloudRGB);
  filter.setInputCloud(cloud);
  filter.setFilterFieldName("z");
  filter.setFilterLimits(pt_min,pt_max);
  filter.filter(*cloud_cropped);

  return cloud_cropped;
}

//Function for LAB-based conditional Euler clustering
pcl::IndicesClustersPtr LABConditionalClustering(CloudRGB::Ptr cloud, pcl::search::KdTree<PointT>::Ptr tree){
  pcl::ConditionalEuclideanClustering<PointT> cec (true);
  pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters);
  cec.setInputCloud (cloud);
  cec.setConditionFunction (&enforceLABDistanceSimilarity);
  cec.setClusterTolerance(cluster_tolerance);
  cec.setMinClusterSize (cloud->size () * cluster_size_perc_min);
  cec.setMaxClusterSize (cloud->size () * cluster_size_perc_max);
  cec.segment (*clusters);

  return clusters;
}

//Function for estimating point cloud normals
pcl::PointCloud<PointN>::Ptr normalEstimation(CloudRGB::Ptr cloud, pcl::search::KdTree<PointT>::Ptr tree){
  pcl::PointCloud<PointN>::Ptr cloud_normals (new pcl::PointCloud<PointN>);
  pcl::NormalEstimation<PointT, PointN> ne;
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud);
  ne.setKSearch (normal_k);
  ne.compute (*cloud_normals);

  return cloud_normals;
}

//Function to determine cylinder score (returns the score in a tuple with cluster cloud, cluster cylinder inliers and cluster cylinder coefficients)
std::tuple<double, pcl::PointCloud<PointT>::Ptr, pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr>
determineClusterFitScore(CloudRGB::Ptr cloud, pcl::PointCloud<PointN>::Ptr cloud_normals, pcl::IndicesClustersPtr clusters, 
                        pcl::SACSegmentationFromNormals<PointT, PointN> seg, int i){
  
  double axis_roll;
  double axis_pitch;
  double axis_yaw;
  double fitting_score = 0.0;
  double point_distance;
  pcl::PointCloud<PointT>::Ptr cluster_cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointN>::Ptr cluster_normals (new pcl::PointCloud<PointN>);
  pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);

  pcl::PointIndices cluster = (*clusters)[i];
  for (const auto& idx : cluster.indices) 
  {
    cluster_cloud->push_back((*cloud)[idx]);
    cluster_normals->push_back((*cloud_normals)[idx]);
  }

  cluster_cloud->header.frame_id = publish_frame;
  
  seg.setInputCloud (cluster_cloud);
  seg.setInputNormals (cluster_normals);  

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);

  if (coefficients_cylinder->values.size()>0)
  {
    axis_roll = atan(coefficients_cylinder->values[5]/coefficients_cylinder->values[4]);
    axis_pitch = -1.0 * atan(coefficients_cylinder->values[5]/coefficients_cylinder->values[3]);
    axis_yaw = atan(coefficients_cylinder->values[4]/coefficients_cylinder->values[3]);
    
    if ((cylinder_min_roll<std::abs(axis_roll) && std::abs(axis_roll)<cylinder_max_roll) && 
        (cylinder_min_pitch<std::abs(axis_pitch) && std::abs(axis_pitch)<cylinder_max_pitch) && 
        (cylinder_min_yaw<std::abs(axis_yaw) && std::abs(axis_yaw)<cylinder_max_yaw))
    {

      if (inliers_cylinder->indices.size() > 0)
      {
        //Score cylinder fit
        Eigen::Vector4f cylinder_axis_point(coefficients_cylinder->values[0], coefficients_cylinder->values[1],
                                    coefficients_cylinder->values[2], 0);
        Eigen::Vector4f cylinder_axis_dir(coefficients_cylinder->values[3], coefficients_cylinder->values[4],
                                    coefficients_cylinder->values[5], 0);
        double cylinder_radius = coefficients_cylinder->values[6];
        double squared_length_axis_dir = cylinder_axis_dir.squaredNorm();
        int cluster_size = cluster.indices.size();

        for (int j = 0; j < cluster_size; ++j)
        {
          Eigen::Vector4f cluster_point((*cluster_cloud)[j].x,(*cluster_cloud)[j].y,(*cluster_cloud)[j].z,0); 
          //Calculate distance from cylinder surface then add point if within cylinder_scoring_distance*cylinder_radius
          point_distance = std::abs(cylinder_radius - sqrt(pcl::sqrPointToLineDistance(cluster_point, cylinder_axis_point, 
                                                                        cylinder_axis_dir, squared_length_axis_dir)));
          if (point_distance <= cylinder_scoring_distance*cylinder_radius)
          {
              fitting_score += 1;
          }
        }
        
        fitting_score /= cluster_size;   
      }
    }
  }

  return std::tuple<double, pcl::PointCloud<PointT>::Ptr, pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr>{
    fitting_score,
    cluster_cloud,
    inliers_cylinder,
    coefficients_cylinder
  };
}

//Fuction to iterate through clusters and return cloud with best perch fit along with respective cylinder inlier coefficients and cylinder model coefficients
std::tuple<pcl::PointCloud<PointT>::Ptr, pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr, float>
findBestCylinderFit(CloudRGB::Ptr cloud, pcl::PointCloud<PointN>::Ptr cloud_normals, pcl::IndicesClustersPtr clusters){
  pcl::SACSegmentationFromNormals<PointT, PointN> seg;    
  seg.setOptimizeCoefficients (optimise_cylinder);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (cylinder_normal_distance);
  seg.setMaxIterations (cylinder_max_iterations);
  seg.setDistanceThreshold (cylinder_inlier_distance);
  seg.setRadiusLimits (cylinder_radius_min, cylinder_radius_max);
  seg.setInputCloud (cloud);
  seg.setInputNormals (cloud_normals);

  pcl::PointCloud<PointT>::Ptr best_cylinder_cloud (new pcl::PointCloud<PointT>);
  best_cylinder_cloud->header.frame_id = publish_frame;
  pcl::ModelCoefficients::Ptr best_coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr best_inliers_cylinder (new pcl::PointIndices);
  
  double best_fit = 0.0;

   for(int i = 0; i < clusters->size (); ++i)
    {
      std::tuple<double, pcl::PointCloud<PointT>::Ptr, pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr>
      cluster_fit_score = determineClusterFitScore(cloud, cloud_normals, clusters, seg, i);

      // Evaluate fit
      // The cluster with the highest score is deemed the best fit
      if (std::get<0>(cluster_fit_score)>best_fit)
      {
        best_fit = std::get<0>(cluster_fit_score);
        best_cylinder_cloud->swap(*std::get<1>(cluster_fit_score));
        best_inliers_cylinder = std::get<2>(cluster_fit_score);
        best_coefficients_cylinder = std::get<3>(cluster_fit_score);
      } 
    }

  return std::tuple<pcl::PointCloud<PointT>::Ptr, pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr, float>{
    best_cylinder_cloud,
    best_inliers_cylinder,
    best_coefficients_cylinder,
    best_fit
  };
}

//Color each cluster in cloud with a different colour with low R value for visualisation
CloudRGB::Ptr colourClusters(CloudRGB::Ptr cloud, pcl::IndicesClustersPtr clusters){
  int r_rand;
  int g_rand;
  int b_rand;

  for (int i = 0; i < clusters->size (); ++i)
  {
    r_rand = rand()%100;
    g_rand = rand()%256;
    b_rand = rand()%256;
    for (int j = 0; j < (*clusters)[i].indices.size (); ++j)
    {
      (*cloud)[(*clusters)[i].indices[j]].r = r_rand;
      (*cloud)[(*clusters)[i].indices[j]].b = b_rand;
      (*cloud)[(*clusters)[i].indices[j]].g = g_rand;
    }
  }

  return cloud;
}

//Colour cylinder cloud red so it stands out in visualisation
CloudRGB::Ptr colourCylinderCloud(CloudRGB::Ptr cloud){
  for (int i = 0; i < cloud->size (); ++i)
  {
      cloud->points[i].r = 255;
      cloud->points[i].g = 0;
      cloud->points[i].b = 0;
  }

  return cloud;
}

//Create marker for cylinder cloud
visualization_msgs::Marker createCylinderMarker(pcl::ModelCoefficients::Ptr best_coefficients_cylinder){
  visualization_msgs::Marker cylinder_marker;
  cylinder_marker.header.frame_id = publish_frame;
  cylinder_marker.header.stamp = ros::Time();
  cylinder_marker.ns = "perch";
  cylinder_marker.id = 0;
  cylinder_marker.type = visualization_msgs::Marker::CYLINDER;
  cylinder_marker.action = visualization_msgs::Marker::ADD;

  cylinder_marker.pose.position.x = best_coefficients_cylinder->values[0];
  cylinder_marker.pose.position.y = best_coefficients_cylinder->values[1];
  cylinder_marker.pose.position.z = best_coefficients_cylinder->values[2];
  
  //Convert axis vector to quarternion format
  double axis_roll = atan2(best_coefficients_cylinder->values[5],best_coefficients_cylinder->values[4]);
  double axis_pitch = -1.0 * atan2(best_coefficients_cylinder->values[5],best_coefficients_cylinder->values[3]);
  double axis_yaw = atan2(best_coefficients_cylinder->values[4],best_coefficients_cylinder->values[3]);
  tf2::Quaternion axis_quarternion;
  axis_quarternion.setRPY( 0.0, -0.5*M_PI + axis_pitch, axis_yaw);
  // axis_quarternion.setRPY( axis_roll, axis_pitch, axis_yaw);
  axis_quarternion.normalize();

  cylinder_marker.pose.orientation.x = axis_quarternion.getX();
  cylinder_marker.pose.orientation.y = axis_quarternion.getY();
  cylinder_marker.pose.orientation.z = axis_quarternion.getZ();
  cylinder_marker.pose.orientation.w = axis_quarternion.getW();
  cylinder_marker.scale.x = 2 * best_coefficients_cylinder->values[6];
  cylinder_marker.scale.y = 2 * best_coefficients_cylinder->values[6];
  cylinder_marker.scale.z = 0.3;
  cylinder_marker.color.a = 0.3; // Don't forget to set the alpha!
  cylinder_marker.color.r = 0.0;
  cylinder_marker.color.g = 1.0;
  cylinder_marker.color.b = 0.0;

  return cylinder_marker;
}

//Extract points from cloud based on point indices
pcl::PointCloud<PointT>::Ptr extractInliers(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointIndices::Ptr indices){
  pcl::PointCloud<PointT>::Ptr inlier_cloud (new pcl::PointCloud<PointT>);
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (indices);
  extract.setNegative (false);
  extract.filter (*inlier_cloud);

  return inlier_cloud;
}

//Create marker for centroid
visualization_msgs::Marker createCentroidMarker(Eigen::Vector4f best_centroid){
  visualization_msgs::Marker centroid_marker;
  centroid_marker.header.frame_id = publish_frame;
  centroid_marker.header.stamp = ros::Time();
  centroid_marker.ns = "centroid";
  centroid_marker.id = 0;
  centroid_marker.type = visualization_msgs::Marker::SPHERE;
  centroid_marker.action = visualization_msgs::Marker::ADD;
  centroid_marker.pose.position.x = best_centroid[0];
  centroid_marker.pose.position.y = best_centroid[1];
  centroid_marker.pose.position.z = best_centroid[2];
  centroid_marker.pose.orientation.x = 0.0;
  centroid_marker.pose.orientation.y = 0.0;
  centroid_marker.pose.orientation.z = 0.0;
  centroid_marker.pose.orientation.w = 1.0;
  centroid_marker.scale.x = 0.1;
  centroid_marker.scale.y = 0.1;
  centroid_marker.scale.z = 0.1;
  centroid_marker.color.a = 1.0; // Don't forget to set the alpha!
  centroid_marker.color.r = 0.0;
  centroid_marker.color.g = 0.0;
  centroid_marker.color.b = 1.0;

  return centroid_marker;
}

//Create Pose message from cylinder coefficients !!EACH FIELD HAS NEW DEFINITION!!
slapper_vision::Perch perchMessageFromCoefficients(pcl::ModelCoefficients::Ptr coefficients_cylinder, float fitting_score){
  //Coefficients can't be sent as a message so we send the coefficient values as a pose message
  // Here we have:
  //      - position is the x,y,and z coordinates of a point on the cylinder axi
  //      - orientation (x,y,z) is the axis direction x,y and z components
  //      - orientation w is the cylinder radius
  slapper_vision::Perch perch_msg;
  perch_msg.axis_point.x = coefficients_cylinder->values[0];
  perch_msg.axis_point.y = coefficients_cylinder->values[1];
  perch_msg.axis_point.z = coefficients_cylinder->values[2];
  
  perch_msg.axis_direction.x = coefficients_cylinder->values[3];
  perch_msg.axis_direction.y = coefficients_cylinder->values[4];
  perch_msg.axis_direction.z = coefficients_cylinder->values[5];
  
  perch_msg.radius = coefficients_cylinder->values[6];

  perch_msg.fitting_score = fitting_score;

  return perch_msg;
}

//-----------------SUBSCRIBER CALLBACK FUNCTIONS--------------------------------------

template<>
void DoublePublisherSingleSubscriber<CloudRGB, CloudRGB, CloudRGB>::subscriberCallback(const CloudRGB::ConstPtr& cloud)
{ 
  CloudRGB::Ptr cloud_downsampled = cloudDownsampling(cloud);

  CloudRGB::Ptr cloud_cropped = cloudPassThrough(cloud_downsampled);

  if (cloud_cropped->size()>0)
  {
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud_cropped);

    pcl::IndicesClustersPtr clusters = LABConditionalClustering(cloud_cropped, tree);
    
    pcl::PointCloud<PointN>::Ptr cloud_normals = normalEstimation(cloud_cropped, tree);

    std::tuple<pcl::PointCloud<PointT>::Ptr, pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr, float>
    best_cylinder_fit_tuple = findBestCylinderFit(cloud_cropped, cloud_normals, clusters);

    //Break up tuple into components
    pcl::PointCloud<PointT>::Ptr best_cylinder_cloud = std::get<0>(best_cylinder_fit_tuple);
    pcl::PointIndices::Ptr best_inliers_cylinder = std::get<1>(best_cylinder_fit_tuple);
    pcl::ModelCoefficients::Ptr best_coefficients_cylinder = std::get<2>(best_cylinder_fit_tuple);

    CloudRGB::Ptr coloured_cloud = colourClusters(cloud_cropped, clusters);
    coloured_cloud->header.frame_id = publish_frame;
    
    // ROS_INFO("PUBLISHING CLUSTERS");
    publisherObject.publish(*coloured_cloud);
    
    // ROS_INFO("Writing File...");
    pcl::io::savePCDFileASCII("/home/seamus/ConvertedClouds/coloured_cluster_test_pcd.pcd",*coloured_cloud);

    CloudRGB::Ptr coloured_cylinder_cloud = colourCylinderCloud(best_cylinder_cloud);

    // ROS_INFO("PUBLISHING COLOURED CYLINDER CLOUD");
    secondPublisherObject.publish(*coloured_cylinder_cloud);
  }
}

template<>
void DoublePublisherSingleSubscriber<CloudRGB, visualization_msgs::Marker, CloudRGB>::subscriberCallback(const CloudRGB::ConstPtr& cloud)
{   
  CloudRGB::Ptr cloud_downsampled = cloudDownsampling(cloud);

  CloudRGB::Ptr cloud_cropped = cloudPassThrough(cloud_downsampled);

  if (cloud_cropped->size()>0)
  {
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud_cropped);

    pcl::IndicesClustersPtr clusters = LABConditionalClustering(cloud_cropped, tree);
    
    pcl::PointCloud<PointN>::Ptr cloud_normals = normalEstimation(cloud_cropped, tree);

    std::tuple<pcl::PointCloud<PointT>::Ptr, pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr, float>
    best_cylinder_fit_tuple = findBestCylinderFit(cloud_cropped, cloud_normals, clusters);

    //Break up tuple into components
    pcl::PointCloud<PointT>::Ptr best_cylinder_cloud = std::get<0>(best_cylinder_fit_tuple);
    pcl::PointIndices::Ptr best_inliers_cylinder = std::get<1>(best_cylinder_fit_tuple);
    pcl::ModelCoefficients::Ptr best_coefficients_cylinder = std::get<2>(best_cylinder_fit_tuple);

    if (cylinder_testing)
    {
      //Publish best cylinder fit
      // ROS_INFO("PUBLISHING");
      publisherObject.publish(*best_cylinder_cloud);

      if (best_cylinder_cloud->size() > 0)
      {
        visualization_msgs::Marker cylinder_marker = createCylinderMarker(best_coefficients_cylinder);
        // Publish cylinder marker
        secondPublisherObject.publish(cylinder_marker);

        // ROS_INFO("Writing File...");
        pcl::io::savePCDFileASCII("/home/seamus/ConvertedClouds/cylinder_test_pcd.pcd",*best_cylinder_cloud);
      } 
    }
    else if (centroid_testing)
    {
      // Extract just the cylinder inliers since this is what we want the centroid of (not the whole cluster)
      pcl::PointCloud<PointT>::Ptr cylinder_inlier_cloud = extractInliers(best_cylinder_cloud, best_inliers_cylinder);
      cylinder_inlier_cloud->header.frame_id = publish_frame;  
      publisherObject.publish(*cylinder_inlier_cloud);
      
      Eigen::Vector4f best_centroid;

      if (pcl::compute3DCentroid(*cylinder_inlier_cloud, best_centroid) != 0)
      {
        visualization_msgs::Marker centroid_marker = createCentroidMarker(best_centroid);
        // Publish centroid marker
        secondPublisherObject.publish(centroid_marker);
      }
    }
  }
}

template<>
void DoublePublisherSingleSubscriber<slapper_vision::Perch, visualization_msgs::Marker, CloudRGB>::subscriberCallback(const CloudRGB::ConstPtr& cloud)
{   
  CloudRGB::Ptr cloud_downsampled = cloudDownsampling(cloud);

  CloudRGB::Ptr cloud_cropped = cloudPassThrough(cloud_downsampled);

  if (cloud_cropped->size()>0)
  {
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud_cropped);

    pcl::IndicesClustersPtr clusters = LABConditionalClustering(cloud_cropped, tree);
    
    pcl::PointCloud<PointN>::Ptr cloud_normals = normalEstimation(cloud_cropped, tree);

    std::tuple<pcl::PointCloud<PointT>::Ptr, pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr, float>
    best_cylinder_fit_tuple = findBestCylinderFit(cloud_cropped, cloud_normals, clusters);

    //Break up tuple into components
    pcl::PointCloud<PointT>::Ptr best_cylinder_cloud = std::get<0>(best_cylinder_fit_tuple);
    pcl::PointIndices::Ptr best_inliers_cylinder = std::get<1>(best_cylinder_fit_tuple);
    pcl::ModelCoefficients::Ptr best_coefficients_cylinder = std::get<2>(best_cylinder_fit_tuple);
    float best_fitting_score = std::get<3>(best_cylinder_fit_tuple);
   
    if (best_cylinder_cloud->size() > 0)
    {
      slapper_vision::Perch perch_msg = perchMessageFromCoefficients(best_coefficients_cylinder, best_fitting_score);
      
      // ROS_INFO("PUBLISHING CYLINDER COEFFICIENTS");
      publisherObject.publish(perch_msg);

      // Publish cylinder marker
      visualization_msgs::Marker cylinder_marker = createCylinderMarker(best_coefficients_cylinder);
      secondPublisherObject.publish(cylinder_marker);
    }
  }
}

//-----------------RECONFIGURE CALLBACK FUNCTION---------------------------------------

//define dynamic reconfigure server callback function
void callback(slapper_vision::cylinder_detectorConfig &config, uint32_t level) {
  //ROS_INFO("Reconfigure Request: %f", config.LAB_threshold);
  LAB_threshold = config.LAB_threshold;
  vg_leaf = config.vg_leaf;
  pt_min = config.pt_min;
  pt_max = config.pt_max;
  cluster_tolerance = config.cluster_tolerance;
  cluster_size_perc_min = config.cluster_size_perc_min;
  cluster_size_perc_max = config.cluster_size_perc_max;
  normal_k = config.normal_k;
  cylinder_normal_distance = config.cylinder_normal_distance;
  cylinder_max_iterations = config.cylinder_max_iterations;
  cylinder_inlier_distance = config.cylinder_inlier_distance;
  cylinder_radius_min = config.cylinder_radius_min;
  cylinder_radius_max = config.cylinder_radius_max;
  cylinder_scoring_distance = config.cylinder_scoring_distance;
  optimise_cylinder = config.optimise_cylinder;
  cylinder_min_roll = pcl::deg2rad(config.cylinder_min_roll);
  cylinder_max_roll = pcl::deg2rad(config.cylinder_max_roll);
  cylinder_min_pitch = pcl::deg2rad(config.cylinder_min_pitch);
  cylinder_max_pitch = pcl::deg2rad(config.cylinder_max_pitch);
  cylinder_min_yaw = pcl::deg2rad(config.cylinder_min_yaw);
  cylinder_max_yaw = pcl::deg2rad(config.cylinder_max_yaw);
}

//-----------------MAIN FUNCTION-------------------------------------------------------

int main(int argc, char **argv)
{
  // ROS_INFO("Node running");
  ros::init(argc, argv, "cylinderDetector");

  //Declare objects for dynamic parameter reconfiguration server and callback with config type
  dynamic_reconfigure::Server<slapper_vision::cylinder_detectorConfig> server;
  dynamic_reconfigure::Server<slapper_vision::cylinder_detectorConfig>::CallbackType f;

  //Set it so that when the server gets a reconfigure request it calls the callback function
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  // Declare node handle which is used only to retrieve ROS params
  ros::NodeHandle nh;
  // Initially set paramater values to those values passed in launch file
  nh.getParam("/cylinder_detector/LAB_threshold", LAB_threshold);
  nh.getParam("/cylinder_detector/vg_leaf", vg_leaf);
  nh.getParam("/cylinder_detector/pt_min", pt_min);
  nh.getParam("/cylinder_detector/pt_max", pt_max);
  nh.getParam("/cylinder_detector/cluster_tolerance", cluster_tolerance);
  nh.getParam("/cylinder_detector/cluster_size_perc_min", cluster_size_perc_min);
  nh.getParam("/cylinder_detector/cluster_size_perc_max", cluster_size_perc_max);
  nh.getParam("/cylinder_detector/normal_k", normal_k);
  nh.getParam("/cylinder_detector/cylinder_normal_distance", cylinder_normal_distance);
  nh.getParam("/cylinder_detector/cylinder_max_iterations", cylinder_max_iterations);
  nh.getParam("/cylinder_detector/cylinder_inlier_distance", cylinder_inlier_distance);
  nh.getParam("/cylinder_detector/cylinder_radius_min", cylinder_radius_min);
  nh.getParam("/cylinder_detector/cylinder_radius_max", cylinder_radius_max);
  nh.getParam("/cylinder_detector/cylinder_scoring_distance", cylinder_scoring_distance);
  nh.getParam("/cylinder_detector/optimise_cylinder", optimise_cylinder);
  nh.getParam("/cylinder_detector/cylinder_min_roll", cylinder_min_roll);
  nh.getParam("/cylinder_detector/cylinder_max_roll", cylinder_max_roll);
  nh.getParam("/cylinder_detector/cylinder_min_pitch", cylinder_min_pitch);
  nh.getParam("/cylinder_detector/cylinder_max_pitch", cylinder_max_pitch);
  nh.getParam("/cylinder_detector/cylinder_min_yaw", cylinder_min_yaw);
  nh.getParam("/cylinder_detector/cylinder_max_yaw", cylinder_max_yaw);
  
  nh.getParam("/cylinder_detector/cluster_testing", cluster_testing);
  nh.getParam("/cylinder_detector/cylinder_testing", cylinder_testing);
  nh.getParam("/cylinder_detector/centroid_testing", centroid_testing);
  nh.getParam("/cylinder_detector/coefficient_testing", coefficient_testing);

  nh.getParam("/cylinder_detector/subscribe_topic", subscribe_topic);
  nh.getParam("/cylinder_detector/publish_frame", publish_frame);

  cylinder_min_roll = pcl::deg2rad(cylinder_min_roll);
  cylinder_max_roll = pcl::deg2rad(cylinder_max_roll);
  cylinder_min_pitch = pcl::deg2rad(cylinder_min_pitch);
  cylinder_max_pitch = pcl::deg2rad(cylinder_max_pitch);
  cylinder_min_yaw = pcl::deg2rad(cylinder_min_yaw);
  cylinder_max_yaw = pcl::deg2rad(cylinder_max_yaw);

  if (cluster_testing)
  {
    first_publish_topic = "perch/cluster";
    second_publish_topic = "perch/cylinder";
    DoublePublisherSingleSubscriber<CloudRGB,CloudRGB,CloudRGB> markerConversion(first_publish_topic,second_publish_topic,
                                                                                subscribe_topic,1);
    
    ros::spin();
  }
  else if (cylinder_testing)
  {
    first_publish_topic = "perch/cylinder";
    second_publish_topic = "perch/cylinder_marker";
    DoublePublisherSingleSubscriber<CloudRGB,visualization_msgs::Marker,CloudRGB> markerConversion(first_publish_topic,second_publish_topic,
                                                                                subscribe_topic,1);

    ros::spin();
  }
  else if (centroid_testing)
  {
    first_publish_topic = "perch/inliers";
    second_publish_topic = "perch/centroid_marker";
    DoublePublisherSingleSubscriber<CloudRGB,visualization_msgs::Marker,CloudRGB> markerConversion(first_publish_topic,second_publish_topic,
                                                                                subscribe_topic,1);

    ros::spin();
  }
  else if (coefficient_testing)
  {
    first_publish_topic = "perch/cylinder_coefficients";
    second_publish_topic = "perch/cylinder_marker";
    DoublePublisherSingleSubscriber<slapper_vision::Perch ,visualization_msgs::Marker,CloudRGB> markerConversion(first_publish_topic,second_publish_topic,
                                                                                subscribe_topic,1);

    ros::spin();
  }

  return 0;
}

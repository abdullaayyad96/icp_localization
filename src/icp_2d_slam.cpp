#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/icp.h>
// #include <pcl/registration/warp_point_rigid_3d.h>
// #include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <string.h>
#include <tf2/convert.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Dense>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include <dynamic_reconfigure/server.h>
#include <icp_2d_slam/ICPCfgConfig.h>
#include <thread>

using namespace std;

//TODO: read parameters from yaml file
#define map_topic "map"
#define odometry_topic "odom"
#define laser_scan_topic "cob_scan_unifier/scan_unified"
#define pose_topic "amcl_pose"
#define map_frame "map"
#define odometry_frame "odom"
#define base_frame "base_link"

//TODO: organize in class 
sensor_msgs::PointCloud2 output_map_cloud, output_scan_cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_map_pointcloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_icp_map_pointcloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_icp_map_pointcloud_crop (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_scan_pointcloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_icp_scan_pointcloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_icp_scan_pointcloud_crop (new pcl::PointCloud<pcl::PointXYZ>);
geometry_msgs::PoseWithCovarianceStamped icp_pose;
pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
Eigen::Matrix4d amcl_transformation_matrix = Eigen::Matrix4d::Identity ();
Eigen::Matrix4d icp_transformation_matrix = Eigen::Matrix4d::Identity ();
Eigen::Vector3d I_amcl_icp_trans; 
Eigen::Matrix3d icp_amcl_rot_mat; 
Eigen::Vector3d M_map_base_trans; 
Eigen::Matrix3d map_base_rot_mat; 
Eigen::Vector3d O_odom_base_trans; 
Eigen::Matrix3d odom_base_rot_mat; 
Eigen::Vector3d M_map_odom_trans; 
Eigen::Matrix3d map_odom_rot_mat; 
bool map_initialized = false;
bool scan_initialized = false;
bool odom_initialized = false;
bool amcl_initialized = true;
bool tf_ready = false;
bool new_amcl_pose = false;
//dynamic parameters
int max_icp_iterations = 10;
double voxel_filter_size = 0.1;
double max_correspondance_distance = 1.0;
double transformation_epsilon = 1e-12;
double crop_filter_size = 5;
double icp_match_thresh = 0.5;
pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
pcl::CropBox<pcl::PointXYZ> crop_filter;
// pcl::registration::WarpPointRigid3D<pcl::PointXYZ, pcl::PointXYZ>::Ptr warp_func (new pcl::registration::WarpPointRigid3D<pcl::PointXYZ, pcl::PointXYZ>);

// pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ>::Ptr transform_estimation (new pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ>);


void callback(icp_2d_slam::ICPCfgConfig &config, uint32_t level) {
  max_icp_iterations = config.max_iterations;
  voxel_filter_size = config.voxel_leaf_size;
  max_correspondance_distance = config.max_correspondance_distance;
  transformation_epsilon = config.transformation_epsilon;
  crop_filter_size = config.crop_filter_size;
  icp_match_thresh = config.max_discrepancy;
  ROS_INFO("Reconfigure Request: %d %f %f %f %f %f", 
            config.max_iterations, config.max_correspondance_distance, 
            config.transformation_epsilon, config.voxel_leaf_size,
            config.max_discrepancy, config.voxel_leaf_size);
}

void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& input_map)
{
  // ROS_INFO("Map received");

  pcl_map_pointcloud->header.frame_id = input_map->header.frame_id;
  pcl_map_pointcloud->height = input_map->info.height;
  pcl_map_pointcloud->width = input_map->info.width;

  pcl::PointXYZ temp_point;
  
  for (int i = 0 ; i <  input_map->info.width *  input_map->info.height ; i++)
  {
    if (input_map->data[i] > 0)
    {
      temp_point.x = (input_map->info.origin.position.x + 0.5 * input_map->info.resolution +  input_map->info.resolution * (i % input_map->info.width));
      temp_point.y = (input_map->info.origin.position.y + 0.5 * input_map->info.resolution + input_map->info.resolution * (i / input_map->info.width));
      temp_point.z = 0.0;
      pcl_map_pointcloud->push_back(temp_point); 
    }  
  }
  voxel_filter.setInputCloud (pcl_map_pointcloud);
  voxel_filter.setLeafSize (voxel_filter_size, voxel_filter_size, voxel_filter_size);
  voxel_filter.filter (*pcl_map_pointcloud);
  //pcl::toROSMsg(*pcl_map_pointcloud, output_map_cloud);
  map_initialized= true;
}

void leser_scanner_callback(const sensor_msgs::LaserScan::ConstPtr& input_scan)
{
  // ROS_INFO("laser scan received");

  pcl_scan_pointcloud->clear();

  pcl_scan_pointcloud->header.frame_id = input_scan->header.frame_id;
  pcl_scan_pointcloud->height = 1;
  pcl_scan_pointcloud->width = input_scan->ranges.size();

  pcl::PointXYZ temp_point;
  
  for (int i = 0 ; i <  input_scan->ranges.size() ; i++)
  {
    if ((input_scan->ranges[i] > input_scan->range_min) && (input_scan->ranges[i] < input_scan->range_max))
    {
      temp_point.x = input_scan->ranges[i] * std::cos(input_scan->angle_min + input_scan->angle_increment * i);
      temp_point.y = input_scan->ranges[i] * std::sin(input_scan->angle_min + input_scan->angle_increment * i);
      temp_point.z = 0.0;
      pcl_scan_pointcloud->push_back(temp_point); 
    }  
  }
  voxel_filter.setInputCloud (pcl_scan_pointcloud);
  voxel_filter.setLeafSize (voxel_filter_size, voxel_filter_size, voxel_filter_size);
  voxel_filter.filter (*pcl_scan_pointcloud);
  //pcl::toROSMsg(*pcl_scan_pointcloud, output_scan_cloud);
  scan_initialized = true;
}

void odometry_callback(const nav_msgs::Odometry::ConstPtr& odometry)
{
  O_odom_base_trans[0] = odometry->pose.pose.position.x;
  O_odom_base_trans[1] = odometry->pose.pose.position.y;
  O_odom_base_trans[2] = odometry->pose.pose.position.z;

  Eigen::Quaterniond q;
  q.x() = odometry->pose.pose.orientation.x;
  q.y() = odometry->pose.pose.orientation.y;
  q.z() = odometry->pose.pose.orientation.z;
  q.w() = odometry->pose.pose.orientation.w;

  odom_base_rot_mat = q.normalized().toRotationMatrix();

  odom_initialized = true;
}

void amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& input_pose)
{
  new_amcl_pose = true;

  amcl_transformation_matrix(0,3) = input_pose->pose.pose.position.x;
  amcl_transformation_matrix(1,3) = input_pose->pose.pose.position.y;
  amcl_transformation_matrix(2,3) = input_pose->pose.pose.position.z;

  Eigen::Quaterniond q;
  q.x() = input_pose->pose.pose.orientation.x;
  q.y() = input_pose->pose.pose.orientation.y;
  q.z() = input_pose->pose.pose.orientation.z;
  q.w() = input_pose->pose.pose.orientation.w;

  amcl_transformation_matrix.block(0,0,3,3) = q.normalized().toRotationMatrix();
  amcl_initialized=true;
  if (map_initialized && scan_initialized && amcl_initialized && odom_initialized)
  {
    Eigen::Matrix4d map_to_base_priori_transformation = Eigen::Matrix4d::Identity();
    if (new_amcl_pose)
    {
      new_amcl_pose = false;
      map_to_base_priori_transformation.block(0,0,3,3) = amcl_transformation_matrix.block(0,0,3,3).transpose();
      map_to_base_priori_transformation.block(0,3,3,1) = - map_to_base_priori_transformation.block(0,0,3,3) * amcl_transformation_matrix.block(0,3,3,1);
    }
    else
    {
      map_to_base_priori_transformation.block(0,0,3,3) = map_odom_rot_mat * odom_base_rot_mat;
      map_to_base_priori_transformation.block(0,3,3,1) = M_map_odom_trans + map_odom_rot_mat * O_odom_base_trans;
    }
    pcl::transformPointCloud(*pcl_map_pointcloud, *pcl_icp_map_pointcloud, map_to_base_priori_transformation);

    //crop pointclouds
    crop_filter.setMin(Eigen::Vector4f(-crop_filter_size, -crop_filter_size, -crop_filter_size, 1.0));
    crop_filter.setMax(Eigen::Vector4f(crop_filter_size, crop_filter_size, crop_filter_size, 1.0));

    crop_filter.setInputCloud(pcl_icp_map_pointcloud);
    crop_filter.filter(*pcl_icp_map_pointcloud_crop);
    pcl_icp_map_pointcloud_crop->header.frame_id = base_frame;
    pcl::toROSMsg(*pcl_icp_map_pointcloud_crop, output_map_cloud);
    
    crop_filter.setInputCloud(pcl_scan_pointcloud);
    crop_filter.filter(*pcl_icp_scan_pointcloud_crop);

    //transform_estimation->setWarpFunction(warp_func);
    //icp.setTransformationEstimation(transform_estimation);
    
    //icp pointcloud
    icp.setInputTarget(pcl_icp_map_pointcloud_crop);
    icp.setInputSource(pcl_icp_scan_pointcloud_crop);
    icp.setMaximumIterations(max_icp_iterations);
    icp.setMaxCorrespondenceDistance(max_correspondance_distance);
    icp.setTransformationEpsilon(transformation_epsilon);
    icp.align(*pcl_icp_scan_pointcloud_crop);
    pcl_icp_scan_pointcloud_crop->header.frame_id = base_frame;
    pcl::toROSMsg(*pcl_icp_scan_pointcloud_crop, output_scan_cloud);

    ROS_INFO("ICP matching score: %d", icp.getFitnessScore());

    if (icp.getFitnessScore() < icp_match_thresh)
    {
      icp_transformation_matrix = icp.getFinalTransformation().cast<double>();

      icp_amcl_rot_mat = icp_transformation_matrix.block(0,0,3,3);
      I_amcl_icp_trans = Eigen::Vector3d(icp_transformation_matrix(0,3), icp_transformation_matrix(1,3), icp_transformation_matrix(2,3));

      map_base_rot_mat = amcl_transformation_matrix.block(0,0,3,3) * icp_amcl_rot_mat;

      Eigen::Vector3d M_amcl_icp_trans = (amcl_transformation_matrix.block(0,3,3,1) + amcl_transformation_matrix.block(0,0,3,3) * I_amcl_icp_trans);
      
      icp_pose.pose.pose.position.x = M_amcl_icp_trans(0);
      icp_pose.pose.pose.position.y = M_amcl_icp_trans(1);
      icp_pose.pose.pose.position.z = M_amcl_icp_trans(2);
      
      M_map_base_trans << icp_pose.pose.pose.position.x, icp_pose.pose.pose.position.y, icp_pose.pose.pose.position.z;

      Eigen::Quaterniond q1(map_base_rot_mat);   
      icp_pose.pose.pose.orientation.x = q1.x();
      icp_pose.pose.pose.orientation.y = q1.y();
      icp_pose.pose.pose.orientation.z = q1.z();
      icp_pose.pose.pose.orientation.w = q1.w();
      
      map_odom_rot_mat = map_base_rot_mat * odom_base_rot_mat.transpose();


      M_map_odom_trans = M_map_base_trans - map_odom_rot_mat * O_odom_base_trans; 
      

      icp_pose.header.frame_id = map_frame;
      tf_ready = true;
    }
  }
}

void ros_callbacks()
{
  dynamic_reconfigure::Server<icp_2d_slam::ICPCfgConfig> server;
  dynamic_reconfigure::Server<icp_2d_slam::ICPCfgConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ros::spin();
}

void publish_tf(ros::NodeHandle ros_node)
{
  //tf
  static tf2_ros::TransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped map_to_odom_Stamped;
  ros::Publisher pose_publisher = ros_node.advertise<geometry_msgs::PoseWithCovarianceStamped>("icp_pose", 3);
  ros::Publisher map_pointcloud_publisher = ros_node.advertise<sensor_msgs::PointCloud2>("map_cloud", 3);
  ros::Publisher scan_pointcloud_publisher = ros_node.advertise<sensor_msgs::PointCloud2>("icp_scan", 3);
  ros::Rate loop_rate(100);    
  while(ros::ok())
  {
    if(tf_ready)
    {           
      try{
        Eigen::Quaterniond q2(map_odom_rot_mat);

        //publish transform
        map_to_odom_Stamped.header.stamp = ros::Time::now();
        map_to_odom_Stamped.header.frame_id = map_frame;
        map_to_odom_Stamped.child_frame_id = odometry_frame;
        map_to_odom_Stamped.transform.translation.x = M_map_odom_trans(0);
        map_to_odom_Stamped.transform.translation.y = M_map_odom_trans(1);
        map_to_odom_Stamped.transform.translation.z = M_map_odom_trans(2);
        
        map_to_odom_Stamped.transform.rotation.x = q2.x();
        map_to_odom_Stamped.transform.rotation.y = q2.y();
        map_to_odom_Stamped.transform.rotation.z = q2.z();
        map_to_odom_Stamped.transform.rotation.w = q2.w();

        static_broadcaster.sendTransform(map_to_odom_Stamped);
        pose_publisher.publish(icp_pose);
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("Could NOT transform base_link to odom: %s", ex.what());
      }

      
    }    
    // if (map_initialized)
    // {    
    //   Eigen::Quaterniond q1(0.9238795, 0, 0, 0.3826834);
    //   Eigen::Matrix4d transformation1 = Eigen::Matrix4d::Identity();
    //   transformation1(0,3) = 3;
    //   transformation1.block(0,0,3,3) = q1.normalized().toRotationMatrix();
    //   cout << "input:" << endl;
    //   cout << transformation1 << endl;
    //   pcl::transformPointCloud(*pcl_map_pointcloud, *pcl_icp_map_pointcloud, transformation1);
    //   pcl_icp_map_pointcloud->header.frame_id = "map";

    //   pcl::toROSMsg(*pcl_icp_map_pointcloud, output_scan_cloud);    

    //   //transform_estimation->setWarpFunction(warp_func);
    //   //icp.setTransformationEstimation(transform_estimation);

    //   icp.setInputTarget(pcl_map_pointcloud);
    //   icp.setInputSource(pcl_icp_map_pointcloud);
    //   icp.setMaxCorrespondenceDistance (1.0);
    //   // Set the maximum number of iterations (criterion 1)
    //   icp.setMaximumIterations (100);
    //   // Set the transformation epsilon (criterion 2)
    //   icp.setTransformationEpsilon (1e-9);
    //   icp.align(*pcl_scan_pointcloud);

    //   cout << "output:" << endl;
    //   cout << icp.getFinalTransformation() << endl;


    //   Eigen::Matrix4d transformation2 = Eigen::Matrix4d::Identity();

    //   transformation2.block(0,3,3,1) = Eigen::Vector3d(5, 5, 0);
      
    //   Eigen::Affine3f crop_transformation(transformation2.cast<float>());
    //   crop_filter.setTransform(crop_transformation.inverse());
    //   crop_filter.setMin(Eigen::Vector4f(-crop_filter_size, -crop_filter_size, -crop_filter_size, 1.0));
    //   crop_filter.setMax(Eigen::Vector4f(crop_filter_size, crop_filter_size, crop_filter_size, 1.0));

    //   crop_filter.setInputCloud(pcl_map_pointcloud);
    //   crop_filter.filter(*pcl_icp_map_pointcloud);
      

    //   pcl::toROSMsg(*pcl_icp_map_pointcloud, output_map_cloud);
    // }

    map_pointcloud_publisher.publish(output_map_cloud);
    scan_pointcloud_publisher.publish(output_scan_cloud);
    loop_rate.sleep();
  }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ICP_SLAM"); 

    ros::NodeHandle ros_node;
    ros::Publisher map_pointcloud_publisher = ros_node.advertise<sensor_msgs::PointCloud2>("icp_map", 3);
    ros::Publisher scan_pointcloud_publisher = ros_node.advertise<sensor_msgs::PointCloud2>("icp_scan", 3);
    
    ros::Subscriber map_sub = ros_node.subscribe(map_topic, 5, map_callback);
    ros::Subscriber scan_sub = ros_node.subscribe(laser_scan_topic, 5, leser_scanner_callback);
    ros::Subscriber odom_sub = ros_node.subscribe(odometry_topic, 5, odometry_callback);
    ros::Subscriber pose_sub = ros_node.subscribe(pose_topic, 5, amcl_pose_callback);

    std::thread process_ros(ros_callbacks);

    std::thread process_tf(publish_tf, ros_node);

    process_ros.join();
    process_tf.join();
    
}
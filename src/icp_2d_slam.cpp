#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <string.h>
#include <tf2/convert.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Dense>


using namespace std;

 
sensor_msgs::PointCloud2 output_map_cloud, output_scan_cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_icp_map_pointcloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_icp_scan_pointcloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
Eigen::Matrix4d amcl_transformation_matrix = Eigen::Matrix4d::Identity ();
Eigen::Matrix4d icp_transformation_matrix = Eigen::Matrix4d::Identity ();
Eigen::Vector3d map_to_intermediate_trans; 
Eigen::Matrix3d intermediate_to_map_rot; 
bool map_initialized = false;
bool scan_initialized = false;

void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& input_map)
{
  //cout << "map received" << endl;

  pcl_icp_map_pointcloud->header.frame_id = input_map->header.frame_id;
  pcl_icp_map_pointcloud->height = input_map->info.height;
  pcl_icp_map_pointcloud->width = input_map->info.width;

  pcl::PointXYZ temp_point;
  
  for (int i = 0 ; i <  input_map->info.width *  input_map->info.height ; i++)
  {
    if (input_map->data[i] > 0)
    {
      temp_point.x = (input_map->info.origin.position.x +  input_map->info.resolution * (i % input_map->info.height));
      temp_point.y = (input_map->info.origin.position.y + input_map->info.resolution * (i / input_map->info.height));
      temp_point.z = 0.0;
      pcl_icp_map_pointcloud->push_back(temp_point); 
    }  
  }
  pcl::toROSMsg(*pcl_icp_map_pointcloud, output_map_cloud);
  map_initialized= true;
}

void leser_scanner_callback(const sensor_msgs::LaserScan::ConstPtr& input_scan)
{
  //cout << "laser scan received" << endl;

  pcl_icp_scan_pointcloud->clear();

  pcl_icp_scan_pointcloud->header.frame_id = input_scan->header.frame_id;
  pcl_icp_scan_pointcloud->height = 1;
  pcl_icp_scan_pointcloud->width = input_scan->ranges.size();

  pcl::PointXYZ temp_point;
  
  for (int i = 0 ; i <  input_scan->ranges.size() ; i++)
  {
    if ((input_scan->ranges[i] > input_scan->range_min) && (input_scan->ranges[i] < input_scan->range_max))
    {
      temp_point.x = input_scan->ranges[i] * std::cos(input_scan->angle_min + input_scan->angle_increment * i);
      temp_point.y = input_scan->ranges[i] * std::sin(input_scan->angle_min + input_scan->angle_increment * i);
      temp_point.z = 0.0;
      pcl_icp_scan_pointcloud->push_back(temp_point); 
    }  
  }
  scan_initialized = true;
}

void amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& input_pose)
{
  cout << "amcl pose received" << endl;

  amcl_transformation_matrix(0,3) = input_pose->pose.pose.position.x;
  amcl_transformation_matrix(1,3) = input_pose->pose.pose.position.y;
  amcl_transformation_matrix(2,3) = input_pose->pose.pose.position.z;

  Eigen::Quaterniond q;
  q.x() = input_pose->pose.pose.orientation.x;
  q.y() = input_pose->pose.pose.orientation.y;
  q.z() = input_pose->pose.pose.orientation.z;
  q.w() = input_pose->pose.pose.orientation.w;

  amcl_transformation_matrix.block(0,0,3,3) = q.normalized().toRotationMatrix();

  if (map_initialized && scan_initialized)
  {
    pcl::transformPointCloud(*pcl_icp_scan_pointcloud, *pcl_icp_scan_pointcloud, amcl_transformation_matrix);
    icp.setInputTarget(pcl_icp_map_pointcloud);
    icp.setInputSource(pcl_icp_scan_pointcloud);
    icp.align(*pcl_icp_scan_pointcloud);
    cout << icp.getFitnessScore() << endl;
    cout << icp.hasConverged() << endl;
    cout << icp.getFinalTransformation() << endl;
    icp_transformation_matrix = icp.getFinalTransformation().cast<double>();
    
    map_to_intermediate_trans = Eigen::Vector3d(icp_transformation_matrix(0,3), icp_transformation_matrix(1,3), icp_transformation_matrix(2,3));
    intermediate_to_map_rot = icp_transformation_matrix.block(0,0,3,3);

    pcl_icp_scan_pointcloud->header.frame_id = "map";
    pcl::toROSMsg(*pcl_icp_scan_pointcloud, output_scan_cloud);
  }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ICP_SLAM"); 

    ros::NodeHandle ros_node;
    ros::Publisher map_pointcloud_publisher = ros_node.advertise<sensor_msgs::PointCloud2>("map_pcl", 3);
    ros::Publisher scan_pointcloud_publisher = ros_node.advertise<sensor_msgs::PointCloud2>("scan_pcl", 3);

    ros::Subscriber map_sub = ros_node.subscribe("map", 100, map_callback);
    ros::Subscriber scan_sub = ros_node.subscribe("scan_unified_filtered", 100, leser_scanner_callback);
    ros::Subscriber pose_sub = ros_node.subscribe("amcl_pose", 100, amcl_pose_callback);

    ros::Rate loop_rate(500);

    while(1)
    {
      loop_rate.sleep();
      ros::spinOnce();
      map_pointcloud_publisher.publish(output_map_cloud);
      scan_pointcloud_publisher.publish(output_scan_cloud);
    }
    
}
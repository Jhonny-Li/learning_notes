#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include "pcl/point_cloud.h"
#include <pcl/common/transforms.h>
std::string map_frame;
std::string base_frame;
std::string imput_pointsclouds;
double min_pointdistance;
double max_pointdistance;
double resolution_param;
double publish_frequency;

ros::Publisher pc_pub;
tf::TransformListener* listener_ptr;
sensor_msgs::PointCloud2 po_final; //全局地图
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_final(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);//临时地图1

geometry_msgs::Transform trans_pose;

#define PCL_THRESH 3.5

struct spcl
{
    float x;
    float y;
    float z;
    float rgb;
};

//geometry_msgs::Quaternion odom_msg_orientation XYZW

void pcCallback(const sensor_msgs::PointCloud2& _msg) {

  // lookup Transform 
  tf::StampedTransform transform;
  try {
    listener_ptr->lookupTransform("/t265_odom_frame", base_frame, ros::Time(0), transform);
    //geometry_msgs/Transform trans_pose  geometry_msgs/Vector3 translation   geometry_msgs/Quaternion rotation
    trans_pose.translation.x = transform.getOrigin().getX();
    trans_pose.translation.y = transform.getOrigin().getY();
    trans_pose.translation.z = transform.getOrigin().getZ();
    trans_pose.rotation.x = transform.getRotation().getX();
    trans_pose.rotation.y = transform.getRotation().getY();
    trans_pose.rotation.z = transform.getRotation().getZ();
    trans_pose.rotation.w = transform.getRotation().getW();
  } catch (tf::TransformException &ex) {
      // ROS_ERROR("[transform pointcloud]%s",ex.what());
    return;
  }
    // Transform to pcl_format
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);//临时地图1
    // pcl::fromROSMsg(_msg, *pcl_cloud);
    // std::cout<<"pcl_cloud->points.size()"<<pcl_final->points.size()<<std::endl;

    sensor_msgs::PointCloud2 trans_pc;
    pcl_ros::transformPointCloud(map_frame, trans_pose,  _msg, trans_pc);//core transform function
    trans_pc.header.frame_id = map_frame;
    trans_pc.header.stamp = _msg.header.stamp;
    pcl::fromROSMsg(trans_pc, *pcl_cloud);



  // voxel filter 用于降采样，否则rviz无法显示过多点云
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
    double resolution = resolution_param;
    voxel_filter.setLeafSize(resolution, resolution, resolution);       // resolution
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp1(new pcl::PointCloud<pcl::PointXYZRGB>);
    voxel_filter.setInputCloud(pcl_cloud);
    voxel_filter.filter(*tmp1);
    tmp1->swap(*pcl_cloud);
}

int main(int argc, char ** argv) {

  // initialize ROS and the node
  ros::init(argc, argv, "trans_pcl");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  //get parameters
  nh_priv.param<std::string>("base_frame_id", base_frame, "/d400_color_optical_frame"); 
  nh_priv.param<std::string>("map_frame_id", map_frame, "/t265_odom_frame");
  nh_priv.param<std::string>("imput_pointsclouds", imput_pointsclouds, "/camera/depth/points");
  //nh_priv.param<double>("min_pointdistance", min_pointdistance, 0.0);
  //nh_priv.param<double>("max_pointdistance", max_pointdistance, 4.0);
  nh_priv.param<double>("resolution", resolution_param, 0.1);
  nh_priv.param<double>("publish_frequency", publish_frequency, 10);

  //获取并发布话题
  ros::Subscriber pc_sub = nh.subscribe(imput_pointsclouds, 1, pcCallback);
  pc_pub = nh.advertise<sensor_msgs::PointCloud2>("trans_pcl", 10);

  // create the listener
  tf::TransformListener listener;
  listener.waitForTransform(map_frame, base_frame, ros::Time(), ros::Duration(1.0));
  listener_ptr = &listener;

  //restrict the callback rate
  ros::Rate rate(publish_frequency);
  while(nh.ok()){
    //回调进行处理
    ros::spinOnce();
    //添加到全局
    //std::vector<spcl> handle;
    std::vector<spcl>  test;
    for(int i=0; i<pcl_cloud->points.size(); i++)
    {
        spcl point_;
        // std::cout<< pc_ros_1.header.stamp - camera_odom.header.stamp<<std::endl;
        // std::cout<<pcl_cloud->points[i].x<<std::endl;
        double distance = pow(pcl_cloud->points[i].x - trans_pose.translation.x, 2)
                                            +pow(pcl_cloud->points[i].y - trans_pose.translation.y, 2)
                                            +pow(pcl_cloud->points[i].z - trans_pose.translation.z, 2);
        distance = pow(distance, 0.5);         
        if(distance < PCL_THRESH)    
         {
            point_.x = pcl_cloud->points[i].x;
            point_.y = pcl_cloud->points[i].y;
            point_.z = pcl_cloud->points[i].z;
            point_.rgb = pcl_cloud->points[i].rgb;
            test.push_back(point_);    
         }
    }
    //test = reduce_x();
    std::cout<<"test.size()"<<test.size()<<std::endl;
    for(int i=0;i<test.size();i++){
      pcl::PointXYZRGB tmp;
      tmp.x = test[i].x;
      tmp.y = test[i].y;
      tmp.z = test[i].z;        
      tmp.rgb = test[i].rgb;
      pcl_final->push_back(tmp);
    }
    test.resize(0);
    //发布
    std::cout<<"pcl_final->points.size()"<<pcl_final->points.size()<<std::endl;
    // Transform to ROSMsg_format
    pcl::toROSMsg(*pcl_final,po_final);
    po_final.header.frame_id = map_frame;
    po_final.header.stamp = ros::Time::now();
    // 发布维护的点云地图
    pc_pub.publish(po_final);
    rate.sleep();
  }
  //ros::spin();
  return EXIT_SUCCESS;
}

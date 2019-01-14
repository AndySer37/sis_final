#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "pcl_ros/point_cloud.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <cstring>
#include <iostream> 
#include <vector>
#include <string>
#include <tf/transform_broadcaster.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include "object_detection/task1out.h"
#include "pose_estimation/pose_estimation.h"

using namespace pcl;
using namespace std;
using namespace cv;

class pose_estimation{
  public:
    pose_estimation();
  private:
    ros::Publisher original_object1_publisher;
    ros::Publisher original_object2_publisher;
    ros::Publisher original_object3_publisher; 
    ros::Publisher downsampled_object_publisher; 
    ros::Publisher denoised_object_publisher; 
    ros::Publisher object_publisher; 
    ros::Subscriber object_mask_sub;
    ros::Subscriber scene_cloud_sub;
    //////////////////For Visualization/////////////////////////////////////////
    int count;
    int total;
    std_msgs::Header CAMERA_FRAME;
    vector < PointCloud<PointXYZRGB>::Ptr, Eigen::aligned_allocator <PointCloud <PointXYZRGB>::Ptr > > object_clouds;
    vector < PointCloud<PointXYZRGB>::Ptr, Eigen::aligned_allocator <PointCloud <PointXYZRGB>::Ptr > > clusters;
    PointCloud<PointXYZRGB>::Ptr scene_cloud;//(new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr original_cloud1;//(new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr original_cloud2;//(new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr original_cloud3;//(new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr downsampled_cloud;//(new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr denoised_cloud;//(new PointCloud<PointXYZRGB>);
    cv_bridge::CvImagePtr cv_ptr;
    /////////////////Functions//////////////////////////
    void update_points(const sensor_msgs::PointCloud2::ConstPtr& cloud);
    void pose_estimation_cb(const sensor_msgs::Image::ConstPtr& mask);
    void object_cloud_filtering(cv_bridge::CvImagePtr mask);
    void point_cloud_preprocessing(PointCloud<PointXYZRGB>::Ptr noised_cloud);
    void point_cloud_pose_estimation(PointCloud<PointXYZRGB>::Ptr sourceCloud, int cl_c);
    void point_cloud_clustering(PointCloud<PointXYZRGB>::Ptr unclustered_cloud);
    void pose_estimation::serviceCb()
};
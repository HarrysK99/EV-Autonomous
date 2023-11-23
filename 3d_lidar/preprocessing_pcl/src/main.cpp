#include <iostream>
#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <set>
#include <pcl/io/pcd_io.h>
#include <boost/format.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <dynamic_reconfigure/server.h>
#include <preprocessing_pcl/preprocessing_pcl_Config.h>

#define Point2 pcl::PointXYZI

typedef pcl::PointXYZI pointTypeIO;

using namespace std;

ros::Publisher pub;
ros::Publisher pub2;
ros::Publisher pub3;
ros::Publisher pub4;
ros::Publisher pub5;

int MAXITERATIONS, MAXDISTANCE_X, MAXDISTANCE_Z;
double DISTANCETHRESHOLD, BOUNDARY;

void callback(preprocessing_pcl::preprocessing_pcl_Config &config, uint32_t level)
{
	MAXITERATIONS = config.MaxIterations;
	DISTANCETHRESHOLD = config.DistanceThreshold;
	BOUNDARY = config.Boundary;
	MAXDISTANCE_X = config.MaxDistance_x;
	MAXDISTANCE_Z = config.MaxDistance_z;
}

void plane_segmentation(const sensor_msgs::PointCloud2::ConstPtr &scan)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>),
		cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>),
		center_x(new pcl::PointCloud<pcl::PointXYZ>),
		center_y(new pcl::PointCloud<pcl::PointXYZ>),
		cloud_1(new pcl::PointCloud<pcl::PointXYZ>),
		cloud_2(new pcl::PointCloud<pcl::PointXYZ>),
		cloud_3(new pcl::PointCloud<pcl::PointXYZ>),
		cloud_4(new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::fromROSMsg(*scan, *cloud);
	pcl::fromROSMsg(*scan, *cloud);

	std::cout << "Input : " << cloud->points.size() << " (" << pcl::getFieldsList(*cloud) << ")" << std::endl;

	// Plane Segmentation===================================

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// seg.setOptimizeCoefficients (true);       //(옵션) // Enable model coefficient refinement (optional).
	seg.setInputCloud(cloud);			   // 입력
	seg.setModelType(pcl::SACMODEL_PLANE); // 적용 모델  // Configure the object to look for a plane.
	seg.setMethodType(pcl::SAC_RANSAC);	   // 적용 방법   // Use RANSAC method.
	// seg.setMaxIterations (MAXITERATIONS);               //최대 실행 수
	seg.setDistanceThreshold(DISTANCETHRESHOLD); // inlier로 처리할 거리 정보   // Set the maximum allowed distance to the model.
	// seg.setRadiusLimits(0, 0.1);     // cylinder경우, Set minimum and maximum radii of the cylinder.
	seg.segment(*inliers, *coefficients); // 세그멘테이션 적용

	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *inliers, *cloud_2);

	pcl::ExtractIndices<pcl::PointXYZ> ec;
	ec.setInputCloud(cloud);
	ec.setIndices(inliers);
	ec.setNegative(true);
	ec.filter(*cloud_3);

	std::cout << "Filtered1 :" << cloud_2->width * cloud_2->height << std::endl;
	std::cout << "Filtered2 :" << cloud_3->width * cloud_3->height << std::endl;

	pcl::PCLPointCloud2 cloud_clustered, cloud_clustered2;
	pcl::toPCLPointCloud2(*cloud_2, cloud_clustered);

	sensor_msgs::PointCloud2 output_clustered, output_clustered2;
	pcl_conversions::fromPCL(cloud_clustered, output_clustered);
	pcl_conversions::fromPCL(cloud_clustered2, output_clustered2);
	output_clustered.header.frame_id = "os_sensor";
	output_clustered2.header.frame_id = "os_sensor";
	// output_clustered.header.frame_id = "fsds/FSCar";
	// output_clustered2.header.frame_id = "fsds/FSCar";
	pub2.publish(output_clustered);
	pub3.publish(output_clustered2);

	//=====================================================

	// ROI 설정=============================================
	pcl::PassThrough<pcl::PointXYZ> pass_x;
	pass_x.setInputCloud(cloud_2);
	pass_x.setFilterFieldName("x");
	pass_x.setFilterLimits(1, MAXDISTANCE_X);
	pass_x.filter(*center_x);

	pcl::PassThrough<pcl::PointXYZ> pass_y;
	pass_y.setInputCloud(center_x);
	pass_y.setFilterFieldName("y");
	pass_y.setFilterLimits(-1 * BOUNDARY, BOUNDARY);
	pass_y.filter(*center_y);

	pcl::PassThrough<pcl::PointXYZ> pass_z;
	pass_y.setInputCloud(center_y);
	pass_y.setFilterFieldName("z");
	pass_y.setFilterLimits(-20, MAXDISTANCE_Z);
	pass_y.filter(*cloud_filtered);

	pcl::PCLPointCloud2 cloud_clustered_center;
	pcl::toPCLPointCloud2(*cloud_filtered, cloud_clustered_center);
	sensor_msgs::PointCloud2 output_clustered_center;
	pcl_conversions::fromPCL(cloud_clustered_center, output_clustered_center);
	output_clustered_center.header.frame_id = "os_sensor";
	// output_clustered_center.header.frame_id = "fsds/FSCar";
	pub4.publish(output_clustered_center);
	//=====================================================
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "preprocessing_pcl");
	ros::NodeHandle nh;

	dynamic_reconfigure::Server<preprocessing_pcl::preprocessing_pcl_Config> server;
	dynamic_reconfigure::Server<preprocessing_pcl::preprocessing_pcl_Config>::CallbackType f;

	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	// ros::Subscriber sub = nh.subscribe("os_cloud_node/points",1,cloud_cb);
	// ros::Subscriber sub= nh.subscribe("os_cloud_node/points",1,input);
	ros::Subscriber sub = nh.subscribe("os_cloud_node/points", 1, plane_segmentation);
	// ros::Subscriber sub = nh.subscribe("fsds/lidar/Lidar1",1,plane_segmentation);
	pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
	pub2 = nh.advertise<sensor_msgs::PointCloud2>("inlier_points", 1);
	pub3 = nh.advertise<sensor_msgs::PointCloud2>("outlier_points", 1);
	pub4 = nh.advertise<sensor_msgs::PointCloud2>("segmentation_outlier_points", 1);
	pub5 = nh.advertise<sensor_msgs::PointCloud2>("final_clustered", 1);

	ros::spin();

	return 0;
}
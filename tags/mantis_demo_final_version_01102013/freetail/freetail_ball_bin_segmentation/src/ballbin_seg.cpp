#include "ros/ros.h"
#include <algorithm>
using namespace std;

#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud.h>
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <boost/foreach.hpp>
#include "ros/console.h"

#include <tf/transform_listener.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include "/opt/ros/fuerte/stacks/pr2_object_manipulation/perception/tabletop_object_detector/srv_gen/cpp/include/tabletop_object_detector/TabletopSegmentation.h"
#include "/home/cgomez/ros/fuerte/swri-ros-pkg/freetail/freetail_ball_bin_segmentation/srv_gen/cpp/include/freetail_ball_bin_segmentation/BallBinSegmentation.h"

class BallBinSegmentor
{

private:
	//! The node handle
	ros::NodeHandle nh_;
	//! Node handle in the private namespace
	ros::NodeHandle priv_nh_;

	//! Publisher for point clouds
	ros::Publisher pub_;

	//
	std::string base_link_;


	//! Service server for ball/bin segmentation
	ros::ServiceServer ballbin_segmentation_srv_;
	ros::ServiceClient seg_srv_;

	//! TF transform listener
	tf::TransformListener listener_;

	//------------------ Callbacks -------------------

	//! Callback for service calls
	bool serviceCallback(freetail_ball_bin_segmentation::BallBinSegmentation::Request &request, freetail_ball_bin_segmentation::BallBinSegmentation::Response &response);

	//------------------ Individual processing steps -------
	void processCloud(const sensor_msgs::PointCloud2 &cloud,
	                    freetail_ball_bin_segmentation::BallBinSegmentation::Response &response);


public:
  //! Subscribes to and advertises topics; initializes fitter and marker publication flags

	BallBinSegmentor(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
	{

    //publish topic for rviz visualization
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("ball_cluster", 100);

    //create the segmentation service server
    ballbin_segmentation_srv_ = nh_.advertiseService(nh_.resolveName("/ballbin_segmentation"),
    		&BallBinSegmentor::serviceCallback, this);

    //create the segmentation service client
    seg_srv_ = nh_.serviceClient<tabletop_object_detector::TabletopSegmentation>("/tabletop_segmentation", true);
    //initialize operational flags
    priv_nh_.param<std::string>("base_link", base_link_, "");

	}

  //! Empty stub
  ~BallBinSegmentor() {}
};

bool BallBinSegmentor::serviceCallback(freetail_ball_bin_segmentation::BallBinSegmentation::Request &request,
		freetail_ball_bin_segmentation::BallBinSegmentation::Response &response)
{
  ros::Time start_time = ros::Time::now();
  std::string topic = nh_.resolveName("/camera/depth_registered/points");
  ROS_INFO("Tabletop detection service called; waiting for a point_cloud2 on topic %s", topic.c_str());
  sensor_msgs::PointCloud2::ConstPtr recent_cloud =
      ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh_, ros::Duration(3.0));
  if (!recent_cloud)
    {
      ROS_ERROR("Freetail ball/bin segmentor: no point_cloud2 has been received");
      response.result = response.NO_CLOUD_RECEIVED;
      return true;
    }
  ROS_INFO_STREAM("Point cloud received after " << ros::Time::now() - start_time << " seconds; begin processing");

  processCloud(*recent_cloud, response);
  //clearOldMarkers(recent_cloud->header.frame_id);

  return true;
}
void BallBinSegmentor::processCloud(const sensor_msgs::PointCloud2 &cloud,
                                     freetail_ball_bin_segmentation::BallBinSegmentation::Response &response)
{
  ROS_INFO("Starting process on new cloud in frame %s", cloud.header.frame_id.c_str());

  //STEP 1
  // Use tabletop_object_detector segmentation service
  tabletop_object_detector::TabletopSegmentation segmentation_srv;
      if (!seg_srv_.call(segmentation_srv))
         {
            ROS_ERROR("Call to segmentation service failed");

          }
      if (segmentation_srv.response.result != segmentation_srv.response.SUCCESS)
          {
            ROS_ERROR("Segmentation service returned error %d", segmentation_srv.response.result);

          }
  ROS_INFO("Segmentation service succeeded. Detected %d clusters", (int)segmentation_srv.response.clusters.size());

  //STEP 2
  //Take tabletop response clusters, and use first one
  sensor_msgs::PointCloud2 bincluster;
  sensor_msgs::PointCloud clustervector;
  clustervector=segmentation_srv.response.clusters[0];
  sensor_msgs::convertPointCloudToPointCloud2(clustervector, bincluster);
  bincluster.header.frame_id=segmentation_srv.response.table.pose.header.frame_id;
  bincluster.header.stamp=segmentation_srv.response.table.pose.header.stamp;
  //pub2.publish (bincluster);

  //STEP 3
  //TAKE CLUSTER OF BIN/BALL AND REMOVE RED (BIN)
  pcl::PointCloud<pcl::PointXYZRGB> PCxyzrgb;
  pcl::fromROSMsg(bincluster,PCxyzrgb);

  ROS_INFO("Cluster of bin with balls - %d points", (int)PCxyzrgb.width);
    //ROS_INFO("Cluster of bin with balls - height %d", (int)PCxyzrgb.height);
    //width - the total number of points in the cloud (equal with POINTS see below) for unorganized datasets
    int pcsize=PCxyzrgb.width;
    std::vector<int> ivec;
    for (int count=0; count < pcsize; count++)
    {
  	  float r1, g1, b1;
  	  r1=PCxyzrgb.at(count).r;
  	  g1=PCxyzrgb.at(count).g;

  	  //convert RGB to HSV
  	  float r, g, b;
  	  r=r1/255.0; g=g1/255.0; b=b1/255.0;
  	  float M, m, C;
  	  M=max(max(r,g),b);
  	  m=min(min(r,g),b);
  	  C=M-m;
	  float I=(r+g+b)/3.0;
	  float S3=1-(m/I);

	  double alpha = 0.5*(2*r-g-b);
	  double beta = (sqrt(3)/2.0)*(g-b);
	  double H2 = atan2(beta, alpha);
	  double C2 = sqrt((alpha*alpha)+(beta*beta));

	  float S2, V2;
	  V2=M;
	  if (C2==0.0)
	  	  {
	  		  S2=0.0;
	  	  }
	  	  else
	  	  {
	  		  S2=C2/V2;
	  	  }

	  if (H2<=1 && H2>=-1 && S2>0.3)
	  {
		  ivec.push_back(count);

	  }

  }
  for (int i=0; i < ivec.size(); i++)
    {
	  PCxyzrgb.erase(PCxyzrgb.begin()+ivec[i]);
    }

  ROS_INFO("Cluster of balls - %d points", (int)PCxyzrgb.width);
  response.result = response.SUCCESS;

  sensor_msgs::PointCloud2 ballcluster;
  pcl::toROSMsg(PCxyzrgb, ballcluster);
  ballcluster.header.frame_id=segmentation_srv.response.table.pose.header.frame_id;
  ballcluster.header.stamp=segmentation_srv.response.table.pose.header.stamp;
  //publish clusters for rviz
  pub_.publish (ballcluster);

  ROS_INFO("Ball Cluster found and published");
  response.ballcluster = ballcluster;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ballbin_segmentation_node");
  ros::NodeHandle nh;

  BallBinSegmentor node(nh);
  ros::spin();
  return 0;
}

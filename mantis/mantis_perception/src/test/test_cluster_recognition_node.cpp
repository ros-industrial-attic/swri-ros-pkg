/*
 * test_cluster_recognition_node.cpp
 *
 *  Created on: Oct 25, 2012
 */

#include <mantis_perception/template_matching/TemplateAlignment.h>
#include <tabletop_object_detector/TabletopSegmentation.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <boost/foreach.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;

const std::string TOPIC_CLOUD_BEST_FIT = "/cloud_best_fit";
const std::string TOPIC_CLOUD_SEGMENTED_CANDIDATE = "/cloud_segmented";
const std::string SEGMENTATION_SERVICE_NAME = "/tabletop_segmentation";
std::string NODE_NAME = "test_cluster_recognition";

class RecognitionTest
{
public:
	RecognitionTest()
	:world_frame_("base_link"),
	 sensor_frame_("camera_link"),
	 templates_directory_(""),
	 template_files_(),
	 normal_radius_(0.01f),
	 feature_radius_(0.01f),
	 min_sample_distance_(0.001f),
	 max_correspondance_distance_(0.01f),//max_correspondance_distance_(0.01f * 0.01f),
	 max_iterations_(200),
	 perform_downsampling_(true),
	 downsampling_voxel_grid_size_(0.005f),
	 use_template_(false),
	 template_index_(0),
	 sensor_transform_()
	{
		ros::NodeHandle nh;
		NODE_NAME = ros::this_node::getName();

		setup();
	}

	~RecognitionTest()
	{

	}

	void fetchParameters(std::string nameSpace = "")
	{
		ros::param::param(NODE_NAME + "/world_frame",world_frame_,world_frame_);
		ros::param::param(NODE_NAME + "/sensor_frame",sensor_frame_,sensor_frame_);
		ros::param::param(NODE_NAME + "/templates_directory",templates_directory_,templates_directory_);
		ros::param::param(NODE_NAME + "/normal_radius",normal_radius_,normal_radius_);
		ros::param::param(NODE_NAME + "/feature_radius",feature_radius_,feature_radius_);
		ros::param::param(NODE_NAME + "/min_sample_distance",min_sample_distance_,min_sample_distance_);
		ros::param::param(NODE_NAME + "/max_correspondance_distance",max_correspondance_distance_,max_correspondance_distance_);
		ros::param::param(NODE_NAME + "/max_iterations",max_iterations_,max_iterations_);
		ros::param::param(NODE_NAME + "/perform_downsampling",perform_downsampling_,perform_downsampling_);
		ros::param::param(NODE_NAME + "/voxel_side",downsampling_voxel_grid_size_,downsampling_voxel_grid_size_);
		ros::param::param(NODE_NAME + "/use_template_as_target",use_template_,use_template_);
		ros::param::param(NODE_NAME + "/template_index",template_index_,template_index_);

		XmlRpc::XmlRpcValue list;
		ros::param::param(NODE_NAME + "/template_files",list,list);
		if(list.getType() == XmlRpc::XmlRpcValue::TypeArray)
		{
			//BOOST_FOREACH(std::pair<std::string,XmlRpc::XmlRpcValue > p, list)
			for(int i = 0; i < list.size(); i++ )
			{
				std::string name = static_cast<std::string>(list[i]);
				template_files_.push_back(name);
			}
		}
		else
		{
			ROS_ERROR_STREAM(NODE_NAME<<": template_files param is not a valid array entry");
		}
	}

	void updateParameters(std::string nameSpace = "")
	{
		ros::param::param(NODE_NAME + "/world_frame",world_frame_,world_frame_);
		ros::param::param(NODE_NAME + "/min_sample_distance",min_sample_distance_,min_sample_distance_);
		ros::param::param(NODE_NAME + "/max_correspondance_distance",max_correspondance_distance_,max_correspondance_distance_);
		ros::param::param(NODE_NAME + "/max_iterations",max_iterations_,max_iterations_);
		ros::param::param(NODE_NAME + "/use_template_as_target",use_template_,use_template_);
		ros::param::param(NODE_NAME + "/template_index",template_index_,template_index_);
	}

	void spin()
	{
		ros::AsyncSpinner spinner(2);
		spinner.start();

		ros::Duration loopTime(1.0f);

		// data recipients.
		tf::StampedTransform clusterTf;
		Eigen::Affine3d mat;
		sensor_msgs::PointCloud2 clusterMsg;
		TemplateAlignment::AlignmentResult result;


		while(!segmentation_client_.waitForExistence(ros::Duration(4.0f)) && ros::ok())
		{
			ROS_INFO_STREAM(NODE_NAME<<": Waiting for segmentation service");
		}

		ROS_INFO_STREAM(NODE_NAME<<": Found segmentation service");

		while(ros::ok())
		{
			// updating parameters
			updateParameters(NODE_NAME);

			// setting template aligment parameters
			template_aligment_.setMaxCorrespondanceDistance(max_correspondance_distance_);
			template_aligment_.setMinSampleDistance(min_sample_distance_);
			template_aligment_.setMaxIterations(max_iterations_);

			if(use_template_ && (template_index_ > -1) && ((unsigned int)template_index_ < template_files_.size()))
			{
				ROS_INFO_STREAM(NODE_NAME<<": Using template file :"<<template_files_[template_index_]);
				template_aligment_.setCandidateModel(template_aligment_.getModelTemplates()[template_index_]);
				if(!template_aligment_.findBestAlignment(result))
				{
					ROS_ERROR_STREAM(NODE_NAME<<": Recognition failed");
					continue;
				}

				const TemplateAlignment::ModelFeatureData &matchedTemplateData =  template_aligment_.getModelTemplates()[result.Index_];
				matched_cluster_ = *matchedTemplateData.PointCloud_;
				pcl::toROSMsg(*template_aligment_.getModelTemplates()[template_index_].PointCloud_,clusterMsg);
				sensor_msgs::convertPointCloud2ToPointCloud(clusterMsg,target_cluster_msg_);
				target_cluster_msg_.header.frame_id = world_frame_;

			}
			else
			{
				// service request
				tabletop_object_detector::TabletopSegmentation::Request req;
				tabletop_object_detector::TabletopSegmentation::Response res;

				// calling tabletop segmentation service
				if(!segmentation_client_.call(req,res))
				{
					ROS_ERROR_STREAM(NODE_NAME<<": Segmentation request failed, skipping recognition");
					continue;
				}
				else
				{
					ROS_INFO_STREAM(NODE_NAME<<": Segmentation request completed, "<<res.clusters.size()<<" cluster found");

				}

				if(res.clusters.empty())
				{
					ROS_ERROR_STREAM(NODE_NAME<<": No cluster were returned from the segmentation, skipping");
					continue;
				}

				// finding transform
				sensor_msgs::PointCloud &cluster = res.clusters[0];
				try
				{
					tf_listener_.lookupTransform(cluster.header.frame_id,world_frame_,ros::Time::now(),clusterTf);
					tf::TransformTFToEigen(clusterTf,mat);
				}
				catch(tf::TransformException &e)
				{
					ROS_ERROR_STREAM(NODE_NAME<<": transform for "<<world_frame_<<" to "<<cluster.header.frame_id<<" not found");
				}

				// converting sensor cloud message and transforming points to world coordinate
				Cloud::Ptr originalCloudPtr = Cloud::Ptr(new Cloud()), cloudPtr = Cloud::Ptr(new Cloud());
				sensor_msgs::convertPointCloudToPointCloud2(cluster,clusterMsg);
				pcl::fromROSMsg(clusterMsg,*originalCloudPtr);

				// transforming cloud points to world coordinates
				pcl::transformPointCloud(*originalCloudPtr,*cloudPtr,Eigen::Affine3f(mat));

				// finding centroid
				Eigen::Vector4f centroid;
				pcl::compute3DCentroid(*originalCloudPtr,centroid);

				// translating cloud to origin
				tf::TransformTFToEigen(tf::Transform(tf::Quaternion::getIdentity(),
						-tf::Vector3(centroid[0],centroid[1],centroid[2])),mat);

				pcl::transformPointCloud(*cloudPtr,*cloudPtr,Eigen::Affine3f(mat));
				//pcl::transformPointCloud(*originalCloudPtr,*cloudPtr,Eigen::Affine3f(mat));

				// storing candidate cluster data
				TemplateAlignment::ModelFeatureData candidateData;
				candidateData.NormalRadius_ = normal_radius_;
				candidateData.FeatureRadius_ = feature_radius_;
				candidateData.ViewPoint_ = tf::Vector3(sensor_transform_.getOrigin());
				candidateData.ModelName_ = "target";
				candidateData.setInputCloud(cloudPtr,perform_downsampling_,downsampling_voxel_grid_size_);
				template_aligment_.setCandidateModel(candidateData);

				if(!template_aligment_.findBestAlignment(result))
				{
					ROS_ERROR_STREAM(NODE_NAME<<": Recognition failed");
					continue;
				}

				// saving results for publishing
				const TemplateAlignment::ModelFeatureData &matchedTemplateData =  template_aligment_.getModelTemplates()[result.Index_];
				tf::TransformTFToEigen(tf::Transform(tf::Quaternion::getIdentity(),
						tf::Vector3(centroid[0],centroid[1],centroid[2]))*result.Transform_,mat);
				matched_cluster_.clear();
				pcl::transformPointCloud(*matchedTemplateData.PointCloud_,matched_cluster_,Eigen::Affine3f(mat));
				target_cluster_msg_ = cluster;
			}

			const TemplateAlignment::ModelFeatureData &data=  template_aligment_.getModelTemplates()[result.Index_];
			std::stringstream resultStream;
			resultStream<<"\nAligment results:\n";
			resultStream<<"\tIndex: "<<result.Index_<<"\n";
			resultStream<<"\tFitness Score: "<<result.FitnessScore_<<"\n";
			resultStream<<"\tModel Name: "<<data.ModelName_<<"\n";
			resultStream<<"\tTemplate Points: "<<data.PointCloud_->size()<<"\n";
			//resultStream<<"\tTarget Points: "<<cloudPtr->size()<<"\n";

			ROS_INFO_STREAM(resultStream.str());


			loopTime.sleep();
		}
	}

protected:

	void setup()
	{
		ros::NodeHandle nh;

		// initializing data
		matched_cluster_.clear();

		// initializing subscribers;
		cloud_template_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(TOPIC_CLOUD_BEST_FIT,1);
		cloud_segmented_publisher_ = nh.advertise<sensor_msgs::PointCloud>(TOPIC_CLOUD_SEGMENTED_CANDIDATE,1);

		// service client
		segmentation_client_ = nh.serviceClient<tabletop_object_detector::TabletopSegmentation>(SEGMENTATION_SERVICE_NAME, true);

		// setting timer
		timer_publisher_ = nh.createTimer(ros::Duration(0.5f),&RecognitionTest::publishCloudstimerCallback,this);

		// getting parameters
		fetchParameters(NODE_NAME);

		if(template_files_.empty())
		{
			ROS_ERROR_STREAM(NODE_NAME<<": no entries found for template models, exiting");
			ros::shutdown();
			return;
		}

		// setting template alignment parameters
		template_aligment_.setMaxCorrespondanceDistance(max_correspondance_distance_);
		template_aligment_.setMinSampleDistance(min_sample_distance_);
		template_aligment_.setMaxIterations(max_iterations_);

		// obtaining sensor's frame
		try
		{
			tf_listener_.lookupTransform(sensor_frame_,world_frame_,ros::Time::now(),sensor_transform_);
		}
		catch(tf::TransformException &e)
		{
			ROS_ERROR_STREAM(NODE_NAME<<": transform from "<<world_frame_<<" to "<<sensor_frame_<<" not found");
		}

		// adding templates to template aligment;
		BOOST_FOREACH(std::string fileName, template_files_)
		{
			Cloud::Ptr cloudPtr = boost::make_shared<Cloud>();
			TemplateAlignment::ModelFeatureData templateData;

			std::string filePath = templates_directory_ + "/" + fileName;
			if(pcl::io::loadPCDFile<pcl::PointXYZ>(filePath,*cloudPtr) == -1)
			{
				ROS_ERROR_STREAM(NODE_NAME<<": could not read pcd file "<<fileName);
			}
			else
			{
				ROS_INFO_STREAM(NODE_NAME<<" found pcd file "<<fileName<<" with "<<cloudPtr->size()<<" points, adding data to template list");
				templateData.NormalRadius_ = normal_radius_;
				templateData.FeatureRadius_ = feature_radius_;
				templateData.ModelName_ = fileName;
				templateData.ViewPoint_ = tf::Vector3(sensor_transform_.getOrigin());
				//templateData.ViewPoint_ = tf::Vector3(0.0f,0.0f,5.0f);
				templateData.setInputCloud(cloudPtr,perform_downsampling_,downsampling_voxel_grid_size_);
				template_aligment_.addModelTemplate(templateData);
			}
		}
	}

	void publishCloudstimerCallback(const ros::TimerEvent &evnt)
	{
		if(matched_cluster_.empty() && matched_cluster_.points.empty())
		{
			ROS_ERROR_STREAM(NODE_NAME<<": no points in matched cloud, skipping publish");
			return;
		}

		sensor_msgs::PointCloud2 matchedClusterMsg;
		pcl::toROSMsg(matched_cluster_,matchedClusterMsg);

		matchedClusterMsg.header.frame_id = world_frame_;
		matchedClusterMsg.header.stamp = ros::Time::now();
		target_cluster_msg_.header.stamp = ros::Time::now();

		cloud_template_publisher_.publish(matchedClusterMsg);
		cloud_segmented_publisher_.publish(target_cluster_msg_);
	}

	// ros parameters

		// template alignment
		double normal_radius_;
		double feature_radius_;
		double min_sample_distance_;
		double max_correspondance_distance_;
		int max_iterations_;

		// transform resolution
		std::string world_frame_;
		std::string sensor_frame_;// use to determine the point cloud's view point

		// template data
		std::vector<std::string> template_files_;
		std::string templates_directory_;
		bool perform_downsampling_;
		double downsampling_voxel_grid_size_;

		// options
		bool use_template_;
		int template_index_;

	// end of ros parameters

	// transform resolution
	tf::TransformListener tf_listener_;
	tf::StampedTransform sensor_transform_;

	// template alignment
	TemplateAlignment template_aligment_;

	// ros topic publishers
	ros::Publisher cloud_template_publisher_;
	ros::Publisher cloud_segmented_publisher_;

	// ros service clients
	ros::ServiceClient segmentation_client_;

	// timers
	ros::Timer timer_publisher_;

	// persistent results
	Cloud matched_cluster_;
	sensor_msgs::PointCloud target_cluster_msg_;

};

int main(int argc,char** argv)
{
	ros::init(argc,argv,"test_cluster_recognition");
	ros::NodeHandle nh;

	RecognitionTest recognition;
	recognition.spin();

	return 0;
}

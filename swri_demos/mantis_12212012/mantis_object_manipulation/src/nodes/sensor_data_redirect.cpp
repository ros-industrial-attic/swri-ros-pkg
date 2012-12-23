// this node subscribes to topics published by a openni sensor and republishes it with modified frame id's
// where applicable

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>


// list of ros parameter names
static const std::string POINT_CLOUD2_FRAME_ID_PARAM = "point_cloud2_frame_id";

// topic names
static const std::string POINT_CLOUD2_TOPIC_IN = "cloud_in";
static const std::string POINT_CLOUD2_TOPIC_OUT = "cloud_out";

class Republish
{
public:
	Republish()
	{
		ros::NodeHandle nh;

		// getting parameters
		fetchParameters();

		// setting up ros subscribers
		point_cloud2_subs_ = nh.subscribe(POINT_CLOUD2_TOPIC_IN,1,&Republish::pointCloud2SubsCallback,this);

		// setting up ros publishers
		point_cloud2_publ_ = nh.advertise<sensor_msgs::PointCloud2>(POINT_CLOUD2_TOPIC_OUT,1);

	}

	virtual ~Republish()
	{

	}

	void spin()
	{
		ros::NodeHandle nh;

		while(ros::ok())
		{
			ros::spin();
		}

	}


	void pointCloud2SubsCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
	{
		sensor_msgs::PointCloud2 cloudOut;
		cloudOut = *msg;
		cloudOut.header.frame_id = point_cloud_2_frame_id_;

		point_cloud2_publ_.publish(cloudOut);
	}

protected:

	void fetchParameters()
	{
		ros::NodeHandle nh;

		nh.getParam(POINT_CLOUD2_FRAME_ID_PARAM,point_cloud_2_frame_id_);
	}

	ros::Subscriber point_cloud2_subs_;
	ros::Publisher point_cloud2_publ_;

	// ros parameters
	std::string point_cloud_2_frame_id_;
};

int main(int argc,char** argv)
{
	ros::init(argc,argv,"sensor_redirect_node");
	ros::NodeHandle nh;

	Republish sensorDataRepublish;
	sensorDataRepublish.spin();
}



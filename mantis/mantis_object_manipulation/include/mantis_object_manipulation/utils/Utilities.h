#ifndef MANTIS_MANIPULATION_UTILS_H_
#define MANTIS_MANIPULATION_UTILS_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <object_manipulation_msgs/Grasp.h>

struct JointConfiguration
{
public:
	JointConfiguration()
	:SideAngles()
	{
		SideAngles.push_back(-0.40f);
		SideAngles.push_back(0.8f);
		SideAngles.push_back(2.5f);
		SideAngles.push_back(1.5f);
		SideAngles.push_back(-0.5f);
		SideAngles.push_back(1.1f);
		SideAngles.push_back(0.6f);
	}

	~JointConfiguration()
	{

	}

	void fetchParameters(std::string nameSpace = "")
	{
		XmlRpc::XmlRpcValue list;
		if(ros::param::get(nameSpace + "/side_position",list))
		{
			if(list.getType()==XmlRpc::XmlRpcValue::TypeArray && list.size() > 0)
			{
				SideAngles.clear();
				for(int i = 0; i < list.size(); i++)
				{
					XmlRpc::XmlRpcValue &val = list[i];
					if(val.getType() == XmlRpc::XmlRpcValue::TypeDouble)
					{
						SideAngles.push_back(static_cast<double>(val));
					}
				}
			}
		}
	}

	std::vector<double> SideAngles;
};

struct GoalLocation
{
public:
	/*
	 * Determines how subsequent goal poses will be generated
	 */
	enum NextLocationGenerationMode
	{
		FIXED = 0,
		SHUFFLE = 1,
		SQUARE_ARRANGEMENT = 2,
		CIRCULAR_ARRANGEMENT = 3,
		SPIRAL_ARRANGEMENT = 4
	};

public:
	GoalLocation()
	:FrameId("base_link"),
	 ChildFrameId("goal"),
	 GoalTransform(),
	 NumGoalCandidates(8),
	 Axis(0,0,1.0f),
	 NextLocationGenMode(FIXED),
	 MinObjectSpacing(0.05f),
	 MaxObjectSpacing(0.08f),
	 PlaceRegionRadius(0.20f)
	{
		tf::Vector3 pos = tf::Vector3(0.5,-0.48,0.1);
		GoalTransform.setOrigin(pos);
		GoalTransform.setRotation(tf::Quaternion::getIdentity());
	}
	~GoalLocation()
	{

	}

	void fetchParameters(std::string nameSpace)
	{
		double x = 0.0f,y = 0.0f,z = 0.0f, angle = GoalTransform.getRotation().getAngle();
		tf::Vector3 pos = GoalTransform.getOrigin(), axis = GoalTransform.getRotation().getAxis();

		// position parameters
		ros::param::param(nameSpace + "/position/x",x,pos.getX());
		ros::param::param(nameSpace + "/position/y",y,pos.getY());
		ros::param::param(nameSpace + "/position/z",z,pos.getZ());
		pos = tf::Vector3(x,y,z);

		// orientation parameters, should be provided in the form of angle-axis
		ros::param::param(nameSpace + "/orientation/axis/x",x,axis.getX());
		ros::param::param(nameSpace + "/orientation/axis/y",y,axis.getY());
		ros::param::param(nameSpace + "/orientation/axis/z",z,axis.getZ());
		ros::param::param(nameSpace + "/orientation/angle",angle,angle);
		axis = tf::Vector3(x,y,z);

		// goal frame
		ros::param::param(nameSpace + "/frame_id",FrameId,FrameId);
		ros::param::param(nameSpace + "/child_frame_id",ChildFrameId,ChildFrameId);

		tf::Transform t = tf::Transform(tf::Quaternion(axis,angle),pos);
		GoalTransform .setData(t);
		GoalTransform.frame_id_ = FrameId;
		GoalTransform.child_frame_id_ = ChildFrameId;

		// number of candidates
		ros::param::param(nameSpace + "/goal_candidates",NumGoalCandidates,NumGoalCandidates);
		if(NumGoalCandidates <= 0)
		{
			NumGoalCandidates = 1;
		}

		// axis for producing additional candidates
		ros::param::param(nameSpace + "/goal_rotation_axis/x",x,Axis.x());
		ros::param::param(nameSpace + "/goal_rotation_axis/y",y,Axis.y());
		ros::param::param(nameSpace + "/goal_rotation_axis/z",z,Axis.z());
		Axis = tf::Vector3(x,y,z).normalized();

		// next goal location generation parameters
		ros::param::param(nameSpace + "/next_location/generation_mode",(int&)NextLocationGenMode,(int&)NextLocationGenMode);
		ros::param::param(nameSpace + "/next_location/min_spacing",MinObjectSpacing,MinObjectSpacing);
		ros::param::param(nameSpace + "/next_location/max_spacing",MaxObjectSpacing,MaxObjectSpacing);
		ros::param::param(nameSpace + "/next_location/place_region_radius",PlaceRegionRadius,PlaceRegionRadius);
	}

	void generateNextLocationCandidates(std::vector<geometry_msgs::PoseStamped> &placePoses)
	{
			switch(NextLocationGenMode)
			{
			case GoalLocation::FIXED:
				createCandidatePosesByRotation(GoalTransform,NumGoalCandidates,Axis,placePoses);
				break;

			case GoalLocation::CIRCULAR_ARRANGEMENT:
				generateNextLocationCircularMode(placePoses);
				break;

			case GoalLocation::SPIRAL_ARRANGEMENT:
				generateNextLocationSpiralMode(placePoses);
				break;

			case GoalLocation::SQUARE_ARRANGEMENT:
				generateNextLocationSquaredMode(placePoses);
				break;

			case GoalLocation::SHUFFLE:
				generateNextLocationShuffleMode(placePoses);
				break;

			default:
				generateNextLocationShuffleMode(placePoses);
				break;
			}

			geometry_msgs::Point p = placePoses[0].pose.position;
			ROS_INFO_STREAM(ros::this_node::getName()<<"Next Place Location at :[ "<<p.x<<", "<<p.y<<", "<<p.z<<"]");
	}

	std::string FrameId;
	std::string ChildFrameId;
	tf::StampedTransform GoalTransform;
	int NumGoalCandidates; // number of total goal transforms, each additional transform is produced by rotating
							// about an axis by a specified angle
	tf::Vector3 Axis; // used in producing additional goal candidates

	// Next goal position generation
	NextLocationGenerationMode NextLocationGenMode; // generation mode flag
	double MinObjectSpacing; // minimum distance between two objects inside goal region as measured from their local origins
	double MaxObjectSpacing; // maximum distance between two objects inside goal region as measured from their local origins
	double PlaceRegionRadius; // radius of circular region that contains all placed objects

protected:
	std::vector<tf::Transform> previous_locations_;

	void createCandidatePosesByRotation(const tf::Transform &startTrans,int numCandidates,tf::Vector3 axis,
			std::vector<geometry_msgs::PoseStamped> &candidatePoses)
	{
		geometry_msgs::PoseStamped pose;
		pose.header.frame_id = GoalTransform.frame_id_;

		// rotate about z axis and apply to original goal pose in order to create candidates;
		double increment = (2 * M_PI)/((double)numCandidates);
		for(int i = 0; i < numCandidates; i++)
		{
			//double ratio = ((double)i)/((double)numCandidates);
			double angle = increment * i - M_PI ;
			tf::Quaternion q = tf::Quaternion(axis,angle);
			tf::Vector3 p = tf::Vector3(0,0,0);
			tf::Transform candidateTransform = startTrans*tf::Transform(q,p);
			tf::poseTFToMsg(candidateTransform,pose.pose);
			candidatePoses.push_back(pose);
		}
	}

	void generateNextLocationShuffleMode(std::vector<geometry_msgs::PoseStamped> &placePoses)
	{
		// previos pose
		tf::Transform &lastTf = previous_locations_.back();

		// next pose
		tf::Transform nextTf;
		if(previous_locations_.size() == 0)
		{
			nextTf = GoalTransform;
		}
		else
		{
			nextTf = tf::Transform(previous_locations_.back());

			// new location variables
			int distanceSegments = 20; // number of possible values between min and max object spacing
			int angleSegments = 8; // number of possible values between 0 and 2pi
			int randVal;
			double distance;// meters
			double angle,angleMin = 0,angleMax = 2*M_PI; // radians
			double ratio;

			// creating new location relative to the last one
			int maxIterations = 100;
			int iter = 0;
			while(iter < maxIterations)
			{
				iter++;

				// computing distance
				randVal = rand()%distanceSegments + 1;
				ratio = (double)randVal/(double)distanceSegments;
				distance = MinObjectSpacing + ratio*(MaxObjectSpacing - MinObjectSpacing);
				tf::Vector3 trans = tf::Vector3(distance,0.0f,0.0f);

				// computing angle
				randVal = rand()%angleSegments + 1;
				ratio = (double)randVal/(double)angleSegments;
				angle = angleMin + ratio*(angleMax - angleMin);
				tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0f,0.0f,1.0f),angle);

				// computing next pose by rotating and translating from last pose
				nextTf = lastTf * tf::Transform(quat,tf::Vector3(0.0f,0.0f,0.0f))*tf::Transform(tf::Quaternion::getIdentity(),trans);

				// checking if located inside place region
				double distFromCenter = (nextTf.getOrigin() - GoalTransform.getOrigin()).length();
				if((distFromCenter + MinObjectSpacing/2) > PlaceRegionRadius) // falls outside place region, try another
				{
					continue;
				}

				// checking for overlaps against objects already in place region
				double distFromObj;
				BOOST_FOREACH(tf::Transform objTf,previous_locations_)
				{
					distFromObj = (objTf.getOrigin() - nextTf.getOrigin()).length();
					if(distFromObj < MinObjectSpacing)// overlap found, try another
					{
						continue;
					}
				}
			}
		}

		// generating candidate poses from next location found
		createCandidatePosesByRotation(nextTf,NumGoalCandidates,Axis,placePoses);

		// storing next location
		previous_locations_.push_back(nextTf);
	}

	void generateNextLocationSquaredMode(std::vector<geometry_msgs::PoseStamped> &placePoses)
	{
		ROS_WARN_STREAM(ros::this_node::getName()<<": Square place mode not implemented, using shuffle place mode");
		generateNextLocationShuffleMode(placePoses);
	}

	void generateNextLocationCircularMode(std::vector<geometry_msgs::PoseStamped> &placePoses)
	{
		ROS_WARN_STREAM(ros::this_node::getName()<<": Circular place mode not implemented, using shuffle place mode");
		generateNextLocationShuffleMode(placePoses);
	}

	void generateNextLocationSpiralMode(std::vector<geometry_msgs::PoseStamped> &placePoses)
	{
		ROS_WARN_STREAM(ros::this_node::getName()<<": Spiral place mode not implemented, using shuffle place mode");
		generateNextLocationShuffleMode(placePoses);
	}
};

#endif

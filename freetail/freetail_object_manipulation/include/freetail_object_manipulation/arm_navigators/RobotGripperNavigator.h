/*
 * SpherePickingRobotNavigator.h
 *
 *  Created on: Oct 19, 2012
 */

#ifndef ROBOTGRIPPERNAVIGATOR_H_
#define ROBOTGRIPPERNAVIGATOR_H_

#include <object_manipulation_tools/robot_navigators/RobotNavigator.h>
#include <perception_tools/segmentation/SphereSegmentation.h>
#include <object_manipulation_tools/manipulation_utils/GraspSequenceValidator.h>

typedef boost::shared_ptr<GraspSequenceValidator> GraspSequencePtr;

class RobotGripperNavigator : public RobotNavigator
{
public:
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

		void generateNextLocationCandidates(std::vector<geometry_msgs::PoseStamped> &placePoses);

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
				std::vector<geometry_msgs::PoseStamped> &candidatePoses);

		void generateNextLocationShuffleMode(std::vector<geometry_msgs::PoseStamped> &placePoses);
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

public:
	RobotGripperNavigator();
	virtual ~RobotGripperNavigator();

	//virtual void run();

	static std::string MARKER_SEGMENTED_OBJECT;
	static std::string SEGMENTATION_NAMESPACE;
	static std::string GOAL_NAMESPACE;
	static std::string JOINT_CONFIGURATIONS_NAMESPACE;

protected:

	virtual void setup();
	virtual bool performSegmentation();
	bool performSphereSegmentation();

	virtual bool performRecognition()
	{
		ROS_WARN_STREAM(NODE_NAME<<": Skipping Recognition");
	}

	virtual void createCandidateGoalPoses(std::vector<geometry_msgs::PoseStamped> &placePoses);

	// move arm methods
	virtual bool moveArmToSide();
	virtual bool moveArmThroughPickSequence();

	// move execution methods
	virtual bool attemptGraspSequence(const std::string& group_name,const GraspSequenceValidator::GraspSequenceDetails& gei);

	// move generation
	void createPickMoveSequence(const object_manipulation_msgs::PickupGoal &pickupGoal,
			const std::vector<object_manipulation_msgs::Grasp> &grasps,
			std::vector<GraspSequenceValidator::GraspSequenceDetails> &graspSequence);

	// callback overrides
	virtual void callbackPublishMarkers(const ros::TimerEvent &evnt)
	{
		RobotNavigator::callbackPublishMarkers(evnt);
	}

protected:

	// ros parameters
	GoalLocation _GoalParameters;
	JointConfiguration _JointConfigurations;

	// segmentation
	SphereSegmentation _SphereSegmentation;

	// grasp move sequence generator
	GraspSequencePtr grasp_sequence_generator_;


};

#endif /* SPHEREPICKINGROBOTNAVIGATOR_H_ */

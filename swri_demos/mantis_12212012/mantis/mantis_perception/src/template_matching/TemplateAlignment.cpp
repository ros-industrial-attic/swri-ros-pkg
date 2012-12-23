/*
 * TemplateAlignment.cpp
 *
 *  Created on: Oct 24, 2012
 *      Author: coky
 */

#include <mantis_perception/template_matching/TemplateAlignment.h>
#include <boost/foreach.hpp>
#include <tf_conversions/tf_eigen.h>

typedef TemplateAlignment::ModelFeatureData FeatureData;
typedef TemplateAlignment::AlignmentResult Result;

TemplateAlignment::TemplateAlignment(float minSampleDistance, float maxCorrespondanceDistance, int maxIterations)
:min_sample_distance_(minSampleDistance),
 max_correspondance_distance_(maxCorrespondanceDistance),
 max_iterations_(maxIterations)
{
	// TODO Auto-generated constructor stub
	sac_initial_alignment_.setMinSampleDistance(min_sample_distance_);
	sac_initial_alignment_.setMaxCorrespondenceDistance(max_correspondance_distance_);
	sac_initial_alignment_.setMaximumIterations(max_iterations_);
}

TemplateAlignment::~TemplateAlignment() {
	// TODO Auto-generated destructor stub
}

void TemplateAlignment::setCandidateModel(TemplateAlignment::ModelFeatureData &candidate)
{
	model_candidate_ = candidate;
	sac_initial_alignment_.setInputTarget(candidate.PointCloud_);
	sac_initial_alignment_.setTargetFeatures(candidate.Features_);
}

void TemplateAlignment::addModelTemplate(TemplateAlignment::ModelFeatureData &templateData)
{
	model_templates_.push_back(templateData);
}

bool TemplateAlignment::findBestAlignment(TemplateAlignment::AlignmentResult &result)
{
	if(model_templates_.empty())
	{
		return false;
	}

	std::vector<Result> results;
	align(model_templates_,results);

	// finding best fit template (lowest score)
	float lowestScore = std::numeric_limits<float>::infinity();
	int index = 0;
	BOOST_FOREACH(Result res,results)
	{
		if(res.FitnessScore_ < lowestScore)
		{
			index = res.Index_;
			lowestScore = res.FitnessScore_;
		}
	}

	result.FitnessScore_ = results[index].FitnessScore_;
	result.Transform_ = results[index].Transform_;
	result.Index_ = index;

	return true;
}

void TemplateAlignment::align(const FeatureData &templateData,Result &result)
{
	ROS_INFO_STREAM(ros::this_node::getName()<<"/TemplateAlignment: Aligning model "<<templateData.ModelName_);

	sac_initial_alignment_.setInputCloud(templateData.PointCloud_);
	sac_initial_alignment_.setSourceFeatures(templateData.Features_);

	pcl::PointCloud<pcl::PointXYZ> outputCloud;
	sac_initial_alignment_.align(outputCloud);

	result.FitnessScore_ = static_cast<float>(sac_initial_alignment_.getFitnessScore(max_correspondance_distance_));
	Eigen::Matrix4f mat = sac_initial_alignment_.getFinalTransformation();
	Eigen::Matrix3f rot = mat.block<3,3>(0,0);
	Eigen::Vector3f pos = mat.block<3,1>(0,3);

	// creating tf Transform
	tf::Transform t;
	tf::Matrix3x3 orient = 	tf::Matrix3x3(rot(0,0),rot(0,1),rot(0,2),
				rot(1,0),rot(1,1),rot(1,2),
				rot(2,0),rot(2,1),rot(2,2));
	t.setBasis(orient);
	t.setOrigin(tf::Vector3(pos(0),pos(1),pos(2)));

	result.Transform_ = t;

	ROS_INFO_STREAM(ros::this_node::getName()<<"/TemplateAlignment: Finished alignment with score "<<result.FitnessScore_);
}

void TemplateAlignment::align(const std::vector<FeatureData> &templates,std::vector<Result> &results)
{
	results.clear();

	for(int i = 0; i < templates.size(); i++)
	{
		Result result;
		align(templates[i],result);
		result.Index_ = i;
		results.push_back(result);
	}
}

std::vector<TemplateAlignment::ModelFeatureData>& TemplateAlignment::getModelTemplates()
{
	return model_templates_;
}

void TemplateAlignment::ModelFeatureData::computeNormals()
{
	ros::NodeHandle nh;
	ROS_INFO_STREAM(ros::this_node::getName()<<"/TemplateAlignment: computing normals");

	Normals_ = boost::make_shared<NormalsCloud>();

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEstimator;
	normEstimator.setInputCloud (PointCloud_);
	normEstimator.setSearchMethod (SearchMethod_);
	normEstimator.setRadiusSearch (NormalRadius_);
	normEstimator.setViewPoint(ViewPoint_.x(),ViewPoint_.y(),ViewPoint_.z());
	normEstimator.compute (*Normals_);

	ROS_INFO_STREAM(ros::this_node::getName()<<"/TemplateAlignment: finished computing normals");
}

void TemplateAlignment::ModelFeatureData::computeFeatures()
{
	ros::NodeHandle nh;
	ROS_INFO_STREAM(ros::this_node::getName()<<"/TemplateAlignment: computing features");

	Features_ = boost::make_shared<LocalFeatures>();

	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfhEstimator;
	fpfhEstimator.setInputCloud (PointCloud_);
	fpfhEstimator.setInputNormals (Normals_);
	fpfhEstimator.setSearchMethod (SearchMethod_);
	fpfhEstimator.setRadiusSearch (FeatureRadius_);
	fpfhEstimator.compute (*Features_);

	ROS_INFO_STREAM(ros::this_node::getName()<<"/TemplateAlignment: finished computing features");
}

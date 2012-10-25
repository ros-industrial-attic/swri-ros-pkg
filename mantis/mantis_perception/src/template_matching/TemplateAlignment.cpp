/*
 * TemplateAlignment.cpp
 *
 *  Created on: Oct 24, 2012
 *      Author: coky
 */

#include <mantis_perception/template_matching/TemplateAlignment.h>
#include <boost/foreach.hpp>

typedef TemplateAlignment::ModelFeatureData FeatureData;
typedef TemplateAlignment::AlignmentResult Result;

TemplateAlignment::TemplateAlignment()
:min_sample_distance_(0.05f),
 max_correspondance_distance_(0.01f * 0.01f),
 max_iterations_(500)
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
	sac_initial_alignment_.setInputCloud(templateData.PointCloud_);
	sac_initial_alignment_.setSourceFeatures(templateData.Features_);

	pcl::PointCloud<pcl::PointXYZ> outputCloud;
	sac_initial_alignment_.align(outputCloud);

	result.FitnessScore_ = static_cast<float>(sac_initial_alignment_.getFitnessScore(max_correspondance_distance_));
	result.Transform_ = sac_initial_alignment_.getFinalTransformation();
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

const std::vector<TemplateAlignment::ModelFeatureData>& TemplateAlignment::getModelTemplates()
{
	return model_templates_;
}

void TemplateAlignment::ModelFeatureData::computeNormals()
{
	Normals_ = boost::make_shared<NormalsCloud>();

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEstimator;
	normEstimator.setInputCloud (PointCloud_);
	normEstimator.setSearchMethod (SearchMethod_);
	normEstimator.setRadiusSearch (NormalRadius_);
	normEstimator.compute (*Normals_);
}

void TemplateAlignment::ModelFeatureData::computeFeatures()
{
	Features_ = boost::make_shared<LocalFeatures>();

	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfhEstimator;
	fpfhEstimator.setInputCloud (PointCloud_);
	fpfhEstimator.setInputNormals (Normals_);
	fpfhEstimator.setSearchMethod (SearchMethod_);
	fpfhEstimator.setRadiusSearch (FeatureRadius_);
	fpfhEstimator.compute (*Features_);
}

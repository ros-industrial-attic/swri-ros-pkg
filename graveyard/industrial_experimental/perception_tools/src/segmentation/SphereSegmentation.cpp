/*
 * SphereSegmentation.cpp
 *
 *  Created on: Sep 19, 2012
 *      Author: jnicho
 */

#include <perception_tools/segmentation/SphereSegmentation.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <boost/assign.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <pcl/common/transforms.h> // allows conversions between ros msgs and pcl types
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <cmath>


SphereSegmentation::SphereSegmentation()
:_Parameters(),
 _LastSphereSegCluster(),
 _LastCoefficients(),
 _LastIndices(),
 _LastSphereSegSuccess(false)
{
	// TODO Auto-generated constructor stub

}

SphereSegmentation::~SphereSegmentation() {
	// TODO Auto-generated destructor stub
}

void SphereSegmentation::setParameters(const SphereSegmentation::Parameters &parameters)
{
	_Parameters = parameters;
}

SphereSegmentation::Parameters SphereSegmentation::getParameters()
{
	return _Parameters;
}

void SphereSegmentation::fetchParameters(std::string nameSpace)
{
	_Parameters.fetchParameters(nameSpace);
}

bool SphereSegmentation::segment(const sensor_msgs::PointCloud &cloudMsg,arm_navigation_msgs::CollisionObject &obj)
{
	// declaring cloud objs and messages
	sensor_msgs::PointCloud2 intermediateCloudMsg;
	sensor_msgs::convertPointCloudToPointCloud2(cloudMsg,intermediateCloudMsg);
	return segment(intermediateCloudMsg,obj);
}

void SphereSegmentation::getSphereCluster(sensor_msgs::PointCloud &cluster)
{
	if(_LastSphereSegSuccess)
	{
		cluster.points.clear();
		sensor_msgs::PointCloud2 tempCloud;
		pcl::toROSMsg(_LastSphereSegCluster,tempCloud);
		sensor_msgs::convertPointCloud2ToPointCloud(tempCloud,cluster);
	}
}

bool SphereSegmentation::segment(const sensor_msgs::PointCloud2 &cloudMsg,arm_navigation_msgs::CollisionObject &obj)
{
	// cloud objs
	Cloud3D cluster = Cloud3D();

	// converting cloud msg to pcl cloud
	pcl::fromROSMsg(cloudMsg,cluster);

	return segment(cluster,obj);
}

bool SphereSegmentation::segment(const std::vector<sensor_msgs::PointCloud> &clusters, arm_navigation_msgs::CollisionObject &obj,
		int &bestClusterIndex)
{
	// find best fitting cloud
	double score = 0.0f;
	bool success = false;
	arm_navigation_msgs::CollisionObject currentObj;
	Cloud3D bestCluster;
	pcl::PointIndices bestIndices;
	pcl::ModelCoefficients bestCoefficients;
	bool bestSeg;

	for(unsigned int i = 0; i < clusters.size(); i++)
	{
		success = segment(clusters[i],currentObj);
		if(success)
		{
			if(_LastSegmentationScore > score)
			{
				bestClusterIndex = i;
				obj.header.frame_id = currentObj.header.frame_id;
				obj.id = currentObj.id;
				obj.poses = currentObj.poses;
				obj.padding = currentObj.padding;
				obj.shapes = currentObj.shapes;
				bestCluster = _LastSphereSegCluster;
				bestIndices = _LastIndices;
				bestCoefficients = _LastCoefficients;
				score = _LastSegmentationScore;

			}
		}
	}

	// saving best cluster found
	_LastSphereSegCluster = bestCluster;
	_LastIndices = bestIndices;
	_LastCoefficients = bestCoefficients;
	_LastSphereSegSuccess = success;

	return success;
}

bool SphereSegmentation::segment(const Cloud3D &cluster,arm_navigation_msgs::CollisionObject &obj)
{
	std::string nodeName = ros::this_node::getName() + "/segmentation";

	// cloud objs
	Cloud3D cloud = Cloud3D();
	pcl::copyPointCloud(cluster,cloud);

	// will perform recognition in world coordinates so cloud points need to be transformed;
	tf::StampedTransform clusterInWorld; clusterInWorld.setIdentity();
	std::string clusterFrameId = cloud.header.frame_id;

	try
	{
		_TfListener.lookupTransform(_Parameters.WorldFrameId,clusterFrameId,ros::Time(0),clusterInWorld);
	}
	catch(tf::TransformException ex)
	{
		ROS_ERROR("%s",std::string(nodeName + " , failed to resolve transform from " +
				_Parameters.WorldFrameId + " to " + clusterFrameId + " \n\t\t" + " tf error msg: " +  ex.what()).c_str());
		ROS_WARN("%s",std::string(nodeName + ": Will use Identity as cluster transform").c_str());
	}

	// transforming cloud points to world coordinates
	Eigen::Affine3d tfEigen;
	tf::TransformTFToEigen(clusterInWorld,tfEigen);
	pcl::transformPointCloud(cloud,cloud,Eigen::Affine3f(tfEigen));

	// filtering bounds
	//filterBounds(cloud);

	// finding top centroid point
	bool centroidFound = false;
	pcl::PointXYZ topCentroid;
	centroidFound = findTopCentroid(cloud,topCentroid);

	// filtering prism
	if(centroidFound)
	{
		filterWithPolygonalPrism(cloud,topCentroid,20,_Parameters.MaxRadius,1.5f*_Parameters.MaxRadius,0.0f);
		ROS_INFO_STREAM(nodeName<<": Polygonal prism extraction found "<< cloud.points.size()<<" points");
	}
	else
	{
		ROS_WARN_STREAM(nodeName<<": did not find top centroid, skipping polygonal prism extraction");
	}

	// declaring sphere segmentation objs
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());// x, y, z, R are the values returned in that order
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

	if(!performSphereSegmentation(cloud,coefficients,inliers))
	{

		// resetting results
		_LastIndices = pcl::PointIndices();
		_LastCoefficients = pcl::ModelCoefficients();
		_LastSphereSegCluster.clear();
		_LastSphereSegSuccess = false;

		ROS_ERROR_STREAM(nodeName<<": insufficient inliers found "<<inliers->indices.size()<<", exiting segmentation");

		return false;
	}
	else
	{
		ROS_INFO_STREAM(nodeName<<": found "<<inliers->indices.size()<<" total inliers");

		// storing segmented sphere cluster
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setIndices(inliers);
		extract.setInputCloud(boost::make_shared<Cloud3D>(cloud));
		extract.setNegative(false);
		_LastSphereSegCluster.clear();
		extract.filter(_LastSphereSegCluster);
		_LastSphereSegCluster.header = cloud.header;

		// storing results
		_LastIndices = *inliers;
		_LastCoefficients = *coefficients;
		_LastSphereSegSuccess = true;
	}

	if(_Parameters.AlignToTopCentroid && centroidFound)
	{
		coefficients->values[0] = topCentroid.x;
		coefficients->values[1] = topCentroid.y;
		coefficients->values[2] = topCentroid.z - coefficients->values[3]; // top z - radius
		ROS_INFO_STREAM(nodeName<<": Aligned to top centroid");
	}
	else
	{
		ROS_WARN_STREAM(nodeName<<": did not find top centroid");
	}

	ROS_INFO_STREAM(nodeName<<": segmentation succeeded");
	createObject(*coefficients,obj);

	return true;
}

bool SphereSegmentation::performSphereSegmentation(const Cloud3D &cloud,pcl::ModelCoefficients::Ptr coefficients,
		pcl::PointIndices::Ptr inliers)
{
	using namespace pcl;

	// pcl objects
	NormalEstimation<PointXYZ,Normal> normalEstm;
	SACSegmentationFromNormals<PointXYZ,Normal> seg;
	ExtractIndices<PointXYZ> extractPoints;
	ExtractIndices<Normal> extractNormals;
	pcl::search::KdTree<PointXYZ>::Ptr searchTree = boost::make_shared< pcl::search::KdTree<PointXYZ> >();

	// pcl dataholders
	PointCloud<Normal>::Ptr cloudNormals(new PointCloud<Normal>());
	Cloud3D::Ptr cloudPtr = boost::make_shared<Cloud3D>(cloud);

	// normal estimation
	normalEstm.setSearchMethod(searchTree);
	normalEstm.setInputCloud(cloudPtr);
	normalEstm.setKSearch(_Parameters.KNearestNeighbors);
	normalEstm.compute(*cloudNormals);

	// sphere segmentation
	seg.setOptimizeCoefficients(true);
	seg.setModelType(SACMODEL_SPHERE);
	seg.setMethodType(SAC_RANSAC);
	seg.setNormalDistanceWeight(_Parameters.NormalDistanceWeight);
	seg.setMaxIterations(_Parameters.MaxIterations);
	seg.setDistanceThreshold(_Parameters.DistanceThreshold);
	seg.setRadiusLimits(0.0f,_Parameters.MaxRadius);
	seg.setInputCloud(cloudPtr);
	seg.setInputNormals(cloudNormals);
	seg.segment(*inliers,*coefficients);

	// computing score
	//_LastSegmentationScore = ((double)inliers->indices.size())/((double)cloud.points.size());
	_LastSegmentationScore = (double)inliers->indices.size(); // using number of inliers as score

	if(inliers->indices.size() == 0 || _LastSegmentationScore < _Parameters.MinFitnessScore)
	{
		//_LastSegmentationScore = 0.0f;
		return false;
	}
	else
	{
		return true;
	}
}

bool SphereSegmentation::findSphereUsingTopPoint(const Cloud3D &cloud,pcl::ModelCoefficients::Ptr coefficients,
		pcl::PointIndices::Ptr inliers)
{
	std::string nodeName = ros::this_node::getName() + "/segmentation";
	std::stringstream stdOut;

	pcl::PointXYZ centroid;
	if(!findTopCentroid(cloud,centroid))
	{
		return false;
	}

	// filling sphere coefficients, the sphere center is assumed to be located below the centroid by a distance equal to the radius
	coefficients->values.clear();
	coefficients->values.push_back(centroid.x);// x
	coefficients->values.push_back(centroid.y);// y
	coefficients->values.push_back(centroid.z - _Parameters.MaxRadius);// z, uses max radius as the sphere radius
	coefficients->values.push_back(_Parameters.MaxRadius);// R

	stdOut << nodeName << ": Found sphere center at: x = " << coefficients->values[0] <<
			", y = " << coefficients->values[1] << ", z = " << coefficients->values[2];
	ROS_INFO("%s",stdOut.str().c_str());stdOut.str("");

	return true;

}

bool SphereSegmentation::findTopCentroid(const Cloud3D &cloud,pcl::PointXYZ &topCentroid)
{
	std::string nodeName = ros::this_node::getName() + "/segmentation";
	std::stringstream stdOut;

	// finding bounding box
	pcl::PointXYZ pointMax, pointMin;
	pcl::getMinMax3D(cloud,pointMin,pointMax);

	// extracting highest point from bounding box
	double maxZ = pointMax.z;
	ROS_INFO_STREAM(nodeName << ": Found Min Point at x: "<<pointMin.x<<", y: "<<pointMin.y<<", z: " << pointMin.z);
	ROS_INFO_STREAM(nodeName<< ": Found Max Point at x: "<<pointMax.x<<", y: "<<pointMax.y<<", z: " << maxZ);

	// ---------------------------------------------------------------------------------------------------------------
	// finding points near or on top plane
	Cloud3D::Ptr planeCloud  = boost::make_shared<Cloud3D>();
	pcl::PassThrough<pcl::PointXYZ> passFilter;
	passFilter.setInputCloud(boost::make_shared<Cloud3D>(cloud));
	passFilter.setFilterFieldName("z");
	passFilter.setFilterLimits(maxZ - std::abs(_Parameters.DistanceThreshold),
			maxZ + std::abs(_Parameters.DistanceThreshold));
	passFilter.filter(*planeCloud);
	if(planeCloud->size() > 0)
	{
		stdOut << nodeName << ": Found " << planeCloud->size()<<" points at a proximity of "
				<<_Parameters.DistanceThreshold << " to top plane";
		ROS_INFO("%s",stdOut.str().c_str());stdOut.str("");

	}
	else
	{
		stdOut<<nodeName<< ": Found no points on top plane, canceling segmentation";
		ROS_INFO("%s",stdOut.str().c_str());stdOut.str("");
		return false;
	}

	// ---------------------------------------------------------------------------------------------------------------
	// projecting filtered points onto top plane
	// defining plane coefficients
	tf::Vector3 normal = tf::Vector3(0.0f,0.0f,1.0f);
	double offset = -normal.dot(tf::Vector3(0.0f,0.0f,maxZ));
	pcl::ModelCoefficients::Ptr planeCoeffs = boost::make_shared<pcl::ModelCoefficients>();
	planeCoeffs->values.resize(4);
	planeCoeffs->values[0] = normal.getX();
	planeCoeffs->values[1] = normal.getY();
	planeCoeffs->values[2] = normal.getZ();
	planeCoeffs->values[3] = offset;
	stdOut << nodeName << ": Created Top Plane with planeCoeffs: " << planeCoeffs->values[0]
		   << ", "	<< planeCoeffs->values[1] << ", " << planeCoeffs->values[2] << ", " << planeCoeffs->values[3];
	ROS_INFO("%s",stdOut.str().c_str());stdOut.str("");

	// projecting points
	Cloud3D::Ptr projectedCloud = boost::make_shared<Cloud3D>();
	pcl::ProjectInliers<pcl::PointXYZ> projectObj;
	projectObj.setModelType(pcl::SACMODEL_PLANE);
	projectObj.setInputCloud(planeCloud);
	projectObj.setModelCoefficients(planeCoeffs);
	projectObj.filter(*projectedCloud);


	// finding all points within a search radius
	/*	This search attempts to find points that belong to the same object while removing those that are not part
	 * of the object of interest but were found to be close enough to the plane.
	*/
	if(_Parameters.MaxRadius > 0) // skip if search radius <= 0 and use all points instead
	{
		bool continueSearch = true;
		unsigned int index = 0;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ> >();
		std::vector<int> indices;
		std::vector<float> sqDistances;
		searchTree->setInputCloud(projectedCloud);

		while(continueSearch)
		{
			int found = searchTree->radiusSearch(projectedCloud->points[index],
					_Parameters.MaxRadius,indices,sqDistances);

			if(found > 0)
			{
				continueSearch = false;
			}
			else
			{
				index++;	std::string nodeName = ros::this_node::getName();
				std::stringstream stdOut;

				// finding bounding box
				pcl::PointXYZ pointMax, pointMin;
				pcl::getMinMax3D(cloud,pointMin,pointMax);

				// extracting highest point from bounding box
				double maxZ = pointMax.z;
				ROS_INFO_STREAM(nodeName << ": Found Min Point at x: "<<pointMin.x<<", y: "<<pointMin.y<<", z: " << pointMin.z);
				ROS_INFO_STREAM(nodeName<< ": Found Max Point at x: "<<pointMax.x<<", y: "<<pointMax.y<<", z: " << maxZ);

				// ---------------------------------------------------------------------------------------------------------------
				// finding points near or on top plane
				Cloud3D::Ptr planeCloud  = boost::make_shared<Cloud3D>();
				pcl::PassThrough<pcl::PointXYZ> passFilter;
				passFilter.setInputCloud(boost::make_shared<Cloud3D>(cloud));
				passFilter.setFilterFieldName("z");
				passFilter.setFilterLimits(maxZ - std::abs(_Parameters.DistanceThreshold),
						maxZ + std::abs(_Parameters.DistanceThreshold));
				passFilter.filter(*planeCloud);
				if(planeCloud->size() > 0)
				{
					stdOut << nodeName << ": Found " << planeCloud->size()<<" points at a proximity of "
							<<_Parameters.DistanceThreshold << " to top plane";
					ROS_INFO("%s",stdOut.str().c_str());stdOut.str("");

				}
				else
				{
					stdOut<<nodeName<< ": Found no points on top plane, canceling segmentation";
					ROS_INFO("%s",stdOut.str().c_str());stdOut.str("");
					return false;
				}

				// ---------------------------------------------------------------------------------------------------------------
				// projecting filtered points onto top plane
				// defining plane coefficients
				tf::Vector3 normal = tf::Vector3(0.0f,0.0f,1.0f);
				double offset = -normal.dot(tf::Vector3(0.0f,0.0f,maxZ));
				pcl::ModelCoefficients::Ptr planeCoeffs = boost::make_shared<pcl::ModelCoefficients>();
				planeCoeffs->values.resize(4);
				planeCoeffs->values[0] = normal.getX();
				planeCoeffs->values[1] = normal.getY();
				planeCoeffs->values[2] = normal.getZ();
				planeCoeffs->values[3] = offset;
				stdOut << nodeName << ": Created Top Plane with planeCoeffs: " << planeCoeffs->values[0]
					   << ", "	<< planeCoeffs->values[1] << ", " << planeCoeffs->values[2] << ", " << planeCoeffs->values[3];
				ROS_INFO("%s",stdOut.str().c_str());stdOut.str("");

				// projecting points
				Cloud3D::Ptr projectedCloud = boost::make_shared<Cloud3D>();
				pcl::ProjectInliers<pcl::PointXYZ> projectObj;
				projectObj.setModelType(pcl::SACMODEL_PLANE);
				projectObj.setInputCloud(planeCloud);
				projectObj.setModelCoefficients(planeCoeffs);
				projectObj.filter(*projectedCloud);


				// finding all points within a search radius
				/*	This search attempts to find points that belong to the same object while removing those that are not part
				 * of the object of interest but were found to be close enough to the plane.
				*/
				if(_Parameters.MaxRadius > 0) // skip if search radius <= 0 and use all points instead
				{
					bool continueSearch = true;
					unsigned int index = 0;
					pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ> >();
					std::vector<int> indices;
					std::vector<float> sqDistances;
					searchTree->setInputCloud(projectedCloud);

					while(continueSearch)
					{
						int found = searchTree->radiusSearch(projectedCloud->points[index],
								_Parameters.MaxRadius,indices,sqDistances);

						if(found > 0)
						{
							continueSearch = false;
						}
						else
						{
							index++;
							if(index == projectedCloud->points.size())
							{
								ROS_ERROR_STREAM(nodeName << ": Did not find points within search radius: " <<
										_Parameters.MaxRadius <<",  exiting");
								return false;
							}
						}
					}

					// extracting points
					pcl::PointIndices::Ptr inliers = boost::make_shared<pcl::PointIndices>();
					inliers->indices = indices;
					pcl::ExtractIndices<pcl::PointXYZ> extractObj;
					extractObj.setInputCloud(projectedCloud);
					extractObj.setIndices(inliers);
					extractObj.setNegative(false);
					extractObj.filter(*projectedCloud);

					stdOut << nodeName << ": Found " << projectedCloud->size() << " points within search radius: " <<
							_Parameters.MaxRadius;
					ROS_INFO("%s",stdOut.str().c_str());stdOut.str("");
				}
				else
				{
					ROS_INFO_STREAM(nodeName<<": search radius <= 0, skipping search and using all points found instead");
				}

				// finding centroid of projected point cloud (should work well for objects with relative degree of symmetry)
				Eigen::Vector4f centroid;
				pcl::compute3DCentroid(*projectedCloud,centroid);
				if(index == projectedCloud->points.size())
				{
					ROS_ERROR_STREAM(nodeName << ": Did not find points within search radius: " <<
							_Parameters.MaxRadius <<",  exiting");
					return false;
				}
			}
		}

		// extracting points
		pcl::PointIndices::Ptr inliers = boost::make_shared<pcl::PointIndices>();
		inliers->indices = indices;
		pcl::ExtractIndices<pcl::PointXYZ> extractObj;
		extractObj.setInputCloud(projectedCloud);
		extractObj.setIndices(inliers);
		extractObj.setNegative(false);
		extractObj.filter(*projectedCloud);

		stdOut << nodeName << ": Found " << projectedCloud->size() << " points within search radius: " <<
				_Parameters.MaxRadius;
		ROS_INFO("%s",stdOut.str().c_str());stdOut.str("");
	}
	else
	{
		ROS_INFO_STREAM(nodeName<<": search radius <= 0, skipping search and using all points found instead");
	}

	// finding centroid of projected point cloud (should work well for objects with relative degree of symmetry)
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*projectedCloud,centroid);

	topCentroid.x = centroid[0];
	topCentroid.y = centroid[1];
	topCentroid.z = centroid[2];

	return true;
}

void SphereSegmentation::filterBounds(Cloud3D &cloud)
{

	std::vector<std::string> filterFields;
	filterFields.push_back("x"),filterFields.push_back("y"),filterFields.push_back("z");
	std::vector<double> filterMins;
	filterMins.push_back(_Parameters.Xmin) ,filterMins.push_back(_Parameters.Ymin),filterMins.push_back(_Parameters.Zmin);
	std::vector<double> filterMaxs;
	filterMaxs.push_back(_Parameters.Xmax),filterMaxs.push_back(_Parameters.Ymax),filterMaxs.push_back(_Parameters.Zmax);

	pcl::PassThrough<pcl::PointXYZ> passFilter;
	passFilter.setInputCloud(boost::make_shared<Cloud3D>(cloud));

	for(unsigned int i = 0; i < filterFields.size(); i++)
	{
		std::string fieldName = filterFields[i];
		double max, min;
		max = filterMaxs[i];
		min = filterMins[i];

		passFilter.setFilterFieldName(fieldName);
		passFilter.setFilterLimits(min,max);
		passFilter.filter(cloud);
	}

}

void SphereSegmentation::filterWithPolygonalPrism(Cloud3D &cloud,pcl::PointXYZ &point,int nSides,double radius,double heightMax,double heightMin)
{
	Cloud3D::Ptr polygonCloudPtr(new Cloud3D());
	Cloud3D::Ptr cloudPtr = boost::make_shared<Cloud3D>(cloud);

	// iterating n times to produce polygon points
	double maxAngle = 2*M_PI;
	radius = 1.20f * radius; // increasing the radius by 20%
	for(int i = 0; i < nSides; i++)
	{
		pcl::PointXYZ vertex;
		vertex.x = radius * cos((i*maxAngle)/((double)nSides)) + point.x;
		vertex.y = radius * sin((i*maxAngle)/((double)nSides)) + point.y;
		vertex.z = point.z;

		polygonCloudPtr->push_back(vertex);
	}

	// creating extractor
	pcl::ExtractPolygonalPrismData<pcl::PointXYZ> extractor;
	pcl::PointIndices::Ptr indicesPtr(new pcl::PointIndices());
	extractor.setInputCloud(cloudPtr);
	extractor.setInputPlanarHull(polygonCloudPtr);
	extractor.setHeightLimits(heightMin,heightMax);
	extractor.segment(*indicesPtr);

	// remove points outside prism
	pcl::ExtractIndices<pcl::PointXYZ> indexExtractor;
	indexExtractor.setInputCloud(cloudPtr);
	indexExtractor.setIndices(indicesPtr);
	indexExtractor.setNegative(false);
	indexExtractor.filter(cloud);

}

void SphereSegmentation::concatenateClouds(const std::vector<sensor_msgs::PointCloud> &clusters,Cloud3D &cluster)
{
	// clearing cloud
	cluster.clear();
	cluster.header.frame_id = clusters.begin()->header.frame_id;

	// concatenating all clouds first
	sensor_msgs::PointCloud2 tempCloudMsg;
	Cloud3D tempCloud = Cloud3D();

	BOOST_FOREACH(sensor_msgs::PointCloud clusterMsg,clusters)
	{
		sensor_msgs::convertPointCloudToPointCloud2(clusterMsg,tempCloudMsg);

		// converting cloud msg to pcl cloud
		pcl::fromROSMsg(tempCloudMsg,tempCloud);

		// concatenating
		cluster += tempCloud;
	}
}

void SphereSegmentation::createObject(const pcl::ModelCoefficients &coeffs,arm_navigation_msgs::CollisionObject &obj)
{
	const std::string nodeName = ros::this_node::getName() + "/segmentation";

	std::string name = "sphere_object";
	obj.id = name;
	obj.header.frame_id = _Parameters.WorldFrameId;
	obj.padding = 0.0f;
	obj.shapes = std::vector<arm_navigation_msgs::Shape>();
	obj.poses = std::vector<geometry_msgs::Pose>();

	// creating shape
	ROS_INFO_STREAM(nodeName<<": creating sphere shape");
	arm_navigation_msgs::Shape shape;
	shape.type = arm_navigation_msgs::Shape::SPHERE;
	shape.dimensions.push_back(coeffs.values[3]); // radius;

	// creating pose (in world coordinates)
	ROS_INFO_STREAM(nodeName<<": creating sphere pose");
	geometry_msgs::Pose pose;
	tf::poseTFToMsg(tf::Transform(tf::Quaternion::getIdentity(),tf::Vector3(coeffs.values[0],
			coeffs.values[1], coeffs.values[2])),pose);

	// storing objs
	obj.shapes.push_back(shape);
	obj.poses.push_back(pose);

}

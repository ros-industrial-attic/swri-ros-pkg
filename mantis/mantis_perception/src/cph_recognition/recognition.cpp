//mantis_object_recognition_node
//based on work done at UT Austin by Brian O'Neil
#include "ros/ros.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <iostream>
#include <fstream>
#include <math.h>
#include <string.h>

#include <visualization_msgs/Marker.h>

#include <boost/filesystem.hpp>

#include "cph.h"
#include "mantis_perception/mantis_recognition.h"
#include "nrg_object_recognition/recognition.h"
#include "tabletop_object_detector/Table.h"

ros::ServiceClient cph_client;
ros::Publisher rec_pub;
ros::Publisher vis_pub;

bool rec_cb(mantis_perception::mantis_recognition::Request &main_request,
            mantis_perception::mantis_recognition::Response &main_response)
{  

  ROS_INFO("Starting mantis recognition");
  ROS_INFO("Number of clusters received in request = %d", (int)main_request.clusters.size());

  nrg_object_recognition::recognition rec_srv;

  sensor_msgs::PointCloud received_cluster;
  received_cluster=main_request.clusters.at(0);
  sensor_msgs::PointCloud2 cluster;
  sensor_msgs::convertPointCloudToPointCloud2(received_cluster, cluster);
  cluster.header.frame_id=main_request.table.pose.header.frame_id;
  cluster.header.stamp=main_request.table.pose.header.stamp;


  rec_srv.request.cluster = cluster;
  rec_srv.request.threshold = 1000;
      
  //Call recognition service
  cph_client.call(rec_srv);
  if (!cph_client.call(rec_srv))
  {
    ROS_ERROR("Call to cph recognition service failed");
  }
  
////////////////////Assign response values/////////////////////////
  main_response.label = rec_srv.response.label;
  main_response.pose = rec_srv.response.pose;
  ROS_WARN_STREAM("Object labeled as "<< main_response.label);
  //Assign id and marker based on label
  visualization_msgs::Marker mesh_marker;
  mesh_marker.type = visualization_msgs::Marker::MESH_RESOURCE;

  std::size_t found;
  std::string label = main_response.label;
  found=label.find_last_of("/");

  if(label.substr(found+1)=="box")
  {
    main_response.model_id=1;
    mesh_marker.mesh_resource = "package://mantis_perception/data/meshes/demo_parts/elec_enblosure.STL";
  }
  else if (label.substr(found+1)=="coupling")
  {
    main_response.model_id=2;
    mesh_marker.mesh_resource = "package://mantis_perception/data/meshes/demo_parts/rubber_coupler_clamps.STL";
  }
  else if (label.substr(found+1)=="pvc_t")
  {
    main_response.model_id=3;
    mesh_marker.mesh_resource = "package://mantis_perception/data/meshes/demo_parts/pvc_t.STL";
  }
  else if (label.substr(found+1)=="white")
  {
    main_response.model_id=4;
    mesh_marker.mesh_resource = "package://mantis_perception/data/meshes/demo_parts/white_plug.stl";
  }
  else if (label.substr(found+1)=="pvc_elbow")
  {
    main_response.model_id=5;
    mesh_marker.mesh_resource = "package://mantis_perception/data/meshes/demo_parts/pvc_elbow.stl";
  }
  else
  {
	main_response.model_id=1;
	ROS_ERROR_STREAM("Object not properly identified");
  }

  main_response.mesh_marker = mesh_marker;
  vis_pub.publish( mesh_marker );

  ROS_INFO("CPH recognition complete");

//////Visualization://////////////////////////////////////////////////////
      //Import pcd file which matches recognition response
      std::stringstream fileName;
      fileName << "/home"<<rec_srv.response.label << "_" << rec_srv.response.pose.rotation << ".pcd";
      //Load and convert file.
      pcl::PointCloud<pcl::PointXYZ>::Ptr trainingMatch (new pcl::PointCloud<pcl::PointXYZ>);

      pcl::io::loadPCDFile(fileName.str(), *trainingMatch);
      //Translate to location:
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*trainingMatch, centroid);
      pcl::demeanPointCloud<pcl::PointXYZ> (*trainingMatch, centroid, *trainingMatch);

      Eigen::Vector3f translate;
      Eigen::Quaternionf rotate;
      translate(0) = rec_srv.response.pose.x;
      translate(1) = rec_srv.response.pose.y;
      translate(2) = rec_srv.response.pose.z;
      rotate.setIdentity();

      pcl::transformPointCloud(*trainingMatch, *trainingMatch, translate, rotate);
      //make into ros message for publlishing
      sensor_msgs::PointCloud2 recognized_cloud;
      pcl::toROSMsg(*trainingMatch, recognized_cloud);
      //Add transform to header
      recognized_cloud.header.frame_id = main_request.table.pose.header.frame_id;
      recognized_cloud.header.stamp=main_request.table.pose.header.stamp;
      //Publish to topic /recognition_result.
      rec_pub.publish(recognized_cloud);
/////////end visualization////////////////////////////////////////////////////

  return true;
}


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "mantis_object_recognition");
  ros::NodeHandle n;  
  
  ros::ServiceServer rec_serv = n.advertiseService("/mantis_object_recognition", rec_cb);
  cph_client = n.serviceClient<nrg_object_recognition::recognition>("/cph_recognition");
  rec_pub = n.advertise<sensor_msgs::PointCloud2>("/recognition_result",1);
  vis_pub = n.advertise<visualization_msgs::Marker>( "matching_mesh_marker", 0 );
  
  ROS_INFO("mantis object detection/recognition node ready!");
  
  ros::spin(); 
}





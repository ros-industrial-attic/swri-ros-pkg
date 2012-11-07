//mantis_object_recognition_node
//based on work done at UT Austin by Brian O'Neil
#include "ros/ros.h"
#include "std_msgs/UInt16.h"

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

#include <flann/flann.h>

#include <boost/filesystem.hpp>

#include "cph.h"
#include "mantis_perception/mantis_recognition.h"
#include "nrg_object_recognition/recognition.h"
#include "tabletop_object_detector/Table.h"

ros::ServiceClient cph_client;
ros::Publisher rec_pub;

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
  //rec_srv.request.cluster = main_request.clusters.at(0);
  rec_srv.request.threshold = 1000;
      
  //Call recognition service
  cph_client.call(rec_srv);
  if (!cph_client.call(rec_srv))
  {
    ROS_ERROR("Call to cph recognition service failed");
  }
  
  //Assign response values
  main_response.label = rec_srv.response.label;
  main_response.pose = rec_srv.response.pose;
  
  if (main_response.label=="coupling")
  {
    main_response.model_id=1;
  }

  else if (main_response.label=="pvc_t") 
  {
    main_response.model_id=2;
  }
  else if (main_response.label=="enclosure")
  {
    main_response.model_id=3;
  }
  else if (main_response.label=="box")
  {
    main_response.model_id=3;
  }
  else if (main_response.label=="plug")
  {
    main_response.model_id=4;
  }
  else if (main_response.label=="white")
  {
    main_response.model_id=4;
  }
  else main_response.model_id=0;


  ROS_INFO("CPH recognition complete");

//////Visualization://////////////////////////////////////////////////////
      //Import pcd file which matches recognition response
      std::stringstream fileName;
      fileName << "data/" << rec_srv.response.label << "_" << rec_srv.response.pose.rotation << ".pcd";
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
  
/*
  std::vector<std::vector<float> > probTable;
  std::map<std::string, int> classMap;
  std::vector<float> pose_dev;
  std::ifstream objectListFile;
  objectListFile.open("classes.list", std::ios::in);
  std::string tempName, objectLine;
  int k = 0;
  std::vector<float> probRow;
  
  
  while(std::getline(objectListFile, objectLine)){
    std::istringstream objectSS(objectLine);
    //Record object name and add to map.
    objectSS >> tempName;
    classMap.insert(std::pair<std::string, int>(tempName, k));
    //Get pose sigma 
    objectSS >> tempName;
    pose_dev.push_back(atof(tempName.c_str()));
    
    //populate probability table (line k;)
    while(objectSS >> tempName){
      probRow.push_back(atof(tempName.c_str()));
    }
    probTable.push_back(probRow);
    probRow.clear();
    k++;  
  }
  if(k != probTable.at(0).size())
    std::cout << "Error: Probability table is not square!" << std::endl;
  
  for(unsigned int i=0; i<k; i++){
   for(unsigned int j=0; j<k; j++){
     std::cout << probTable.at(i).at(j) << " ";
   }
   std::cout << std::endl;
  }
  std::cout << probTable.size() << " objects in set.\n";
*/
  ROS_INFO("mantis object detection/recognition node ready!");
  
  ros::spin(); 
}





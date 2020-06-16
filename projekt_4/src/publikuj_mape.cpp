#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <fstream>
#include "shape_msgs/Mesh.h"
#include "shape_msgs/MeshTriangle.h"

//do uzyskiwania sciezki do projektu // https://wiki.ros.org/Packages#C.2B-.2B-
#include <ros/package.h>

//do markerow -> start, punkt docelowy
#include "visualization_msgs/Marker.h"

// std_msgs int8
#include "std_msgs/Int8MultiArray.h"

#include <pcl_conversions/pcl_conversions.h> //powoduje blad
#include <pcl/octree/octree_pointcloud.h>

//tutaj definiujesz polozenie punktow: startowego i doclelowego
// double X Y Z
double punkt_startowy[3] = {0, 1.7, 1};
double punkt_docelowy[3] = {-6.5, -1.3, 1};

//tworzenie markerow do punktu startowego i docelowego
visualization_msgs::Marker markerStart;
visualization_msgs::Marker markerDocelowy;

int main(int argc, char** argv)
{
   //sprawdzanie sciezki do projektu -> do wczytania mapy
  std::string path_to_map = ros::package::getPath("projekt_4");

  // definiowanie sciezki do mapy .bt -> //mapa zewnetrzna res = 0.05m
  std::string inputFilename;// = "/home/ubuntu/MAPR_projekt_catkin/src/projekt_4/src/fr_078_tidyup.bt";
  inputFilename = path_to_map + "/maps/fr_078_tidyup.bt";

  std::cout << "\nWczytywanie pliku OcTree!\n\n";
  //zmienna do odczytu OcTree
  std::ifstream file(inputFilename, std::ios_base::in |std::ios_base::binary);
  //jesli nie udalo sie wczytac
  if (!file.is_open())
  {
    OCTOMAP_ERROR_STR("Nie udalo sie wczytac pliku ze sciezki: "<< inputFilename << " BLAD!");
    std::cout << "error -1" << std::endl;
    exit(-1);
  }

  //mapa zewnetrzna
  octomap::OcTree tree(0.05); //res = 0.05 m
  double map_res = 0.05;
  tree.readBinary(file); //wczytaj mape z pliku

  //inicjalizacja node'a, ktory publikuje mape i markery
  ros::init(argc, argv, "projekt_4_publikujMape");
  ros::NodeHandle nh;
  ros::Rate loop_rate(1);

  //create new pointCloud - PointXYZ
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud(new pcl::PointCloud <pcl::PointXYZI>);
  std::cout << "pointCloud.size() = " << pointCloud->size() << std::endl;

  /* PUNKT STARTOWY I DOCELOWY - do publikowania */
  ros::Publisher pub_start = nh.advertise<visualization_msgs::Marker>( "PunktStartowy", 0 );
  ros::Publisher pub_docelowy = nh.advertise<visualization_msgs::Marker>( "PunktDocelowy", 0 );

  markerStart.header.frame_id = "world";
  markerStart.header.stamp = ros::Time();
  markerStart.ns = "markers";
  markerStart.id = 0;
  markerStart.type = visualization_msgs::Marker::CUBE;
  markerStart.action = visualization_msgs::Marker::ADD;
  markerStart.pose.position.x = punkt_startowy[0];
  markerStart.pose.position.y = punkt_startowy[1];
  markerStart.pose.position.z = punkt_startowy[2];
  markerStart.pose.orientation.x = 0.0;
  markerStart.pose.orientation.y = 0.0;
  markerStart.pose.orientation.z = 0.0;
  markerStart.pose.orientation.w = 1.0;
  markerStart.scale.x = .5;
  markerStart.scale.y = .5;
  markerStart.scale.z = .5;
  markerStart.color.a = .6; // Don't forget to set the alpha!
  markerStart.color.r = 0.0;
  markerStart.color.g = 0.0;
  markerStart.color.b = 1.0;

  //DOCELOWY

  markerDocelowy.header.frame_id = "world";
  markerDocelowy.header.stamp = ros::Time();
  markerDocelowy.ns = "markers";
  markerDocelowy.id = 0;
  markerDocelowy.type = visualization_msgs::Marker::CUBE;
  markerDocelowy.action = visualization_msgs::Marker::ADD;
  markerDocelowy.pose.position.x = punkt_docelowy[0];
  markerDocelowy.pose.position.y = punkt_docelowy[1];
  markerDocelowy.pose.position.z = punkt_docelowy[2];
  markerDocelowy.pose.orientation.x = 0.0;
  markerDocelowy.pose.orientation.y = 0.0;
  markerDocelowy.pose.orientation.z = 0.0;
  markerDocelowy.pose.orientation.w = 1.0;
  markerDocelowy.scale.x = .5;
  markerDocelowy.scale.y = .5;
  markerDocelowy.scale.z = .5;
  markerDocelowy.color.a = .6; // Don't forget to set the alpha!
  markerDocelowy.color.r = 1.0;
  markerDocelowy.color.g = 0.0;
  markerDocelowy.color.b = 0.0;
  //publikacje w petli na koncu
  /* PUNKT STARTOWY I DOCELOWY END*/

  // zmienne pomocnicze do tworzenia podlogi na mapie
  double min_x = 1000;
  double max_x = 0;
  double min_y = 1000;
  double max_y = 0;
  double min_z = 1000;
  double max_z = 0;

  // wczytaj mape z pliku do pointCloud formatu PCL
  for(octomap::OcTree::leaf_iterator it = tree.begin(), end=tree.end(); it!= end; ++it)
  {
    if(tree.isNodeOccupied(*it))
    {
      double size = it.getSize();
      //write position to PointXYZ
      pcl::PointXYZI punkt;
      punkt.x = it.getX();
      punkt.y = it.getY();
      punkt.z = it.getZ();
      punkt.intensity = it.getDepth();

      //szukaj min, max dla x, y
      if (punkt.x > max_x)
          max_x = punkt.x;
      if (punkt.x < min_x)
          min_x = punkt.x;

      //dla y
      if (punkt.y > max_y)
          max_y = punkt.y;
      if (punkt.y < min_y)
          min_y = punkt.y;
      //dla z
      if (punkt.z > max_z)
          max_z = punkt.z;
      if (punkt.z < min_z)
          min_z = punkt.z;

      //push pointXYZ to PointCloud
      pointCloud->push_back(punkt);
    }
  }

  //dodawanie podlogi do mapy na calej przestrzeni
  for (int x = min_x/map_res; (x < (max_x / map_res)); x++ )
  {
      for (int y = min_y/map_res; (y < (max_y/map_res)); y++  )
      {
                pcl::PointXYZI punkt;
                punkt.x = x*map_res;
                punkt.y = y*map_res;
                punkt.z = 0;
                punkt.intensity = 0;

                pointCloud->push_back(punkt);
      }
  }

  // Zamiana PCL -> ROS MSG
  // Declare message to publish (http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html)
  sensor_msgs::PointCloud2 cloudToPublish;

  // Convert created point cloud to ROS message
  pcl::toROSMsg(*pointCloud , cloudToPublish);

   //nazwa frejma
  cloudToPublish.header.frame_id = "world";

  // Advertise topic with filtered data - used to publish on a topic
  ros::Publisher filteredPointClouddPub;
  filteredPointClouddPub = nh.advertise<sensor_msgs::PointCloud2>("/Octomapa", 1000);

  // Publish message
  ros::Rate r(1); // 1 Hz

  // publikowanie mapy i markerow
  while (ros::ok())
  {
    filteredPointClouddPub.publish(cloudToPublish);
    //punkty start i docelowy
    pub_docelowy.publish(markerDocelowy);
    pub_start.publish(markerStart);

    ros::spinOnce();
    r.sleep();
  } //while (ros::ok())

  return 0;
}

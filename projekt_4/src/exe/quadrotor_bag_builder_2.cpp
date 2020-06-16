/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <ros/ros.h>

//model drona
#include <rosbag/bag.h>
#include <xpp_msgs/RobotStateJoint.h>
#include <xpp_states/convert.h>
#include <xpp_states/state.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

//wczytywanie mapy
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <unistd.h>
// standard
#include <mutex>
#include <iostream>
#include <thread>
#include <iomanip>
#include <fstream>
#include <iostream>

// Boost
#include <boost/bind.hpp>
#include <boost/thread/recursive_mutex.hpp>

// STL
#include <string>
#include <math.h>
#include <limits>

//pobieranie markerow
#include "visualization_msgs/Marker.h"

//planowanie ruchu - OMPL
#include <ompl/config.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/ConnectionStrategy.h>
#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <nav_msgs/Path.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

using namespace xpp;

ros::Subscriber sledzMape;

// visualize the state of a drone (only has a base)
State3d base;


//callback, ktory sledzi mape - raczej nie bedzie tutaj potrzebny bo chyba
//dron sie nie rusza jak on dziala...

// zmienne pomocnicze do tworzenia macierzy INTow z mapy
double min_x = 1000;
double max_x = 0;
double min_y = 1000;
double max_y = 0;
double min_z = 1000;
double max_z = 0;

//macierz mapy INTow
int* macierzMapy;
int sizeX = 0;
int sizeY = 0;
int sizeZ = 0;
bool mapaWczytana = false;

float map_res = 0.05;

void sledzMape_Callback(const sensor_msgs::PointCloud2ConstPtr &pointCloudMsg)
 {
    //otrzymujemy mape w postaci PointCloud2, konwertujemy ja do PCL'owego PointCloud

    //chmura punktow PCL PointXYZI
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud(new pcl::PointCloud <pcl::PointXYZI>);
    pcl::fromROSMsg(*pointCloudMsg, *pointCloud);

    // dla wszystkich punktow z chmury
    for (int i = 0; i < pointCloud->points.size();i++)
    {
        pcl::PointXYZI punkt = pointCloud->points[i];

        //szukaj min, max dla x, y, z
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
    }
    //mamy juz rzeczywiste wymiary mapy, tworzymy macierz INTow

    /**/
    //przesuwam min_x do 0 i robie odpowiednia ilosc komorek
    sizeX = ceil((double(max_x - min_x)/map_res));
    sizeY = ceil((double(max_y - min_y)/map_res));
    sizeZ = ceil((double(max_z - min_z)/map_res));

    //tworzenie macierzy mapy
    macierzMapy = new int[sizeX * sizeY * sizeZ];

    // ustaw macierzMapy na zera
    for (int i = 0; i < sizeX; i++)
        for (int j = 0; j < sizeY; j++)
            for (int k = 0; k < sizeZ; k++)
            {
                macierzMapy[i*sizeY*sizeZ + j*sizeZ + k] = 0;
            }

    //zapisz w macierzy ktore komorki mapy sa zajete
    for (int i = 0; i < pointCloud->points.size();i++)
    {
        pcl::PointXYZI punkt = pointCloud->points[i];
        double tmp_x;
        double tmp_y;
        double tmp_z;

        tmp_x = punkt.x;
        tmp_y = punkt.y;
        tmp_z = punkt.z;

        int aktualny_x = int((tmp_x - min_x) / map_res);
        int aktualny_y = int((tmp_y - min_y) / map_res);
        int aktualny_z = int((tmp_z - min_z) / map_res);

        macierzMapy[aktualny_x*sizeY*sizeZ + aktualny_y*sizeZ + aktualny_z] = 100; //ustaw jako zajete
    }

    //jesli wczytano mape -> ustawia zmienna zeby nie wczytywac mapy kilka razy
    if (pointCloud->size() > 0)
    {
        std::cout << "sledzMape_Callback - odczytano mape! \n";
        mapaWczytana = true;
    }
 } //sledz_mape Callback

//wczytywanie punktu startowego (x,y,z) z markera
float x_start = 0.0;
float y_start = 0.0;
float z_start = 0.0;
bool punkt_startowy_wczytany = false;

void punktStartowy_callback(const visualization_msgs::Marker markerStart)
{
    x_start = markerStart.pose.position.x;
    y_start = markerStart.pose.position.y;
    z_start = markerStart.pose.position.z;
    //sprawdzenie czy wczytano np. przez odczyt frame
    if (markerStart.header.frame_id == "world")
    {
        punkt_startowy_wczytany = true;
    }
}

//wczytywanie punktu docelowego
float x_docelowy = 0.0;
float y_docelowy = 0.0;
float z_docelowy = 0.0;
bool punkt_docelowy_wczytany = false;

void punktDocelowy_callback(const visualization_msgs::Marker markerDocelowy)
{
    x_docelowy = markerDocelowy.pose.position.x;
    y_docelowy = markerDocelowy.pose.position.y;
    z_docelowy = markerDocelowy.pose.position.z;
    //sprawdzenie czy wczytano np. przez odczyt frame
    if (markerDocelowy.header.frame_id == "world")
    {
        punkt_docelowy_wczytany = true;
    }
}

/*  OMPL    */
namespace ob = ompl::base;
namespace og = ompl::geometric;

/// problem dim
int dim;

/// max step length
double maxStepLength;

/// bounds for the x axis
std::shared_ptr<ompl::base::RealVectorBounds> coordXBound;

/// bounds for the y axis
std::shared_ptr<ompl::base::RealVectorBounds> coordYBound;

/// bounds for the z axis
std::shared_ptr<ompl::base::RealVectorBounds> coordZBound;

/// start position
std::shared_ptr<ompl::base::ScopedState<>> start;

/// goal position
std::shared_ptr<ompl::base::ScopedState<>> goal;

/// search space
std::shared_ptr<ompl::base::StateSpace> space;

bool isStateValid(const ompl::base::State *state)
{

    // get x coord of the robot
    const auto *coordX = state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(0);
    // get y coord of the robot
    const auto *coordY = state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(1);
    // get z coord of the robot
    const auto *coordZ = state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(2);

    //mam zmienne coordX, coordY, coordZ - daja typ rzeczywisty. Ja musze miec zwracane komorki mapy
    int aktualny_x = int((coordX->values[0] - min_x) / map_res);
    int aktualny_y = int((coordX->values[1] - min_y) / map_res);
    int aktualny_z = int((coordX->values[2] - min_z) / map_res);

//rozmiar drona
    //X = 0.5 m -> 10 komorek
    //Y = 0.5 m
    //Z = 0.3 m
int szerokosc_robota = 6;
int wysokosc_robota = 4;
    for (int x = aktualny_x-szerokosc_robota; x < aktualny_x+szerokosc_robota; x++)
    {
        for (int y = aktualny_y-szerokosc_robota; y < aktualny_y+szerokosc_robota; y++)
        {
                for (int z = aktualny_z-wysokosc_robota; z < aktualny_z+wysokosc_robota; z++)
                {
                    //ograniczenia mapy -> nie mozemy sprawdzac poza mapa (indeksy!)
                    if ( (x < 0) || ( y < 0) || ( z < 0) ||
                         ( x >= sizeX) || ( y >= sizeY) || ( z >= sizeZ))
                        return false;

                    if (macierzMapy[(x)*sizeY*sizeZ + (y)*sizeZ + (z)] >= 50) //jesli zajete
                    {
                        //std::cout << "aktualny_x - " << x << " |aktualny_y - " << y << " |aktualny_z - " << z << std::endl;
                        //std::cout << "kolizja: X = " << coordX->values[0] << " Y = " << coordX->values[1] << " Z = " << coordX->values[2] << std::endl;
                        return false;
                    }
                } //end z
        } //end y
    } //end x
    //else
    return true;
}



/*  PUBLIKUJ SCIEZKE    */
trajectory_msgs::MultiDOFJointTrajectory msg_traj;
trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg;
nav_msgs::Path path_nav_msgs;
ros::Publisher path_publisher;
/*  PUBLIKUJ SCIEZKE END    */



#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
 {
     return std::make_shared<ob::PathLengthOptimizationObjective>(si);
 }

ob::OptimizationObjectivePtr allocateObjective(const ob::SpaceInformationPtr& si)
 {
             return getPathLengthObjective(si);
 }


int main(int argc, char *argv[])
{
  // 5x wyszukaj sciezke
  for (int counter = 0; counter < 5; counter++)
  {
      //zmienne do definiowania sciezki do .bag
      int max_rozmiar_sciezki = 10000; //tak na oko. raczej powinno byc zrobiona jako tablica dynamiczna
      double sciezka_x[max_rozmiar_sciezki];
      double sciezka_y[max_rozmiar_sciezki];
      double sciezka_z[max_rozmiar_sciezki];

      //zeruj sciezke
      for (int i=0; i < max_rozmiar_sciezki; i++)
      {
          sciezka_x[i] = 0;
          sciezka_y[i] = 0;
          sciezka_z[i] = 0;
      }
      int dlugosc_sciezki = 0;

      // creates a bag file wherever executable is run (usually ~/.ros/)
      rosbag::Bag bag;
      //tworzy plik .bag, z ktorego chyba korzysta caly pakiet xpp
      bag.open("quadrotor_traj.bag", rosbag::bagmode::Write);

      //inicjalizuje node ROSowy
      ros::init(argc, argv, "quadrotor_sterowanie");
      ros::NodeHandle nh;

      //do sledzenia mapy
      sledzMape = nh.subscribe<sensor_msgs::PointCloud2>("/Octomapa", 1000, sledzMape_Callback);
      //dopoki nie odczytano mapy - to czekaj az bedzie wczytana
      while (mapaWczytana == false)
      {
          ros::spinOnce();
      }

      std::cout << "Rozmiar macierzy Mapy:\n";
      std::cout << "X = " << sizeX << std::endl;
      std::cout << "Y = " << sizeY << std::endl;
      std::cout << "Z = " << sizeZ << std::endl;
      std::cout << std::endl;

      //wczytaj jeszcze punkt startowy i docelowy -> zdefiniowane w publikuj_mape.cpp
      ros::Subscriber punkt_startowy_sub = nh.subscribe<visualization_msgs::Marker>( "PunktStartowy", 0, punktStartowy_callback );
      while (punkt_startowy_wczytany == false)
      {
          ros::spinOnce();
      }
      ros::Subscriber punkt_docelowy_sub = nh.subscribe<visualization_msgs::Marker>( "PunktDocelowy", 0, punktDocelowy_callback);
      while (punkt_docelowy_wczytany == false)
      {
          ros::spinOnce();
      }

      std::cout << "Punkt startowy XYZ:\n";
      std::cout << "X = " << x_start << std::endl;
      std::cout << "Y = " << y_start << std::endl;
      std::cout << "Z = " << z_start << std::endl;

      std::cout << std::endl;
      std::cout << "Punkt docelowy XYZ:\n";
      std::cout << "X = " << x_docelowy << std::endl;
      std::cout << "Y = " << y_docelowy << std::endl;
      std::cout << "Z = " << z_docelowy << std::endl;
      std::cout << std::endl;


      /*    PLANOWANIE RUCHU    */
      // problem dim
      dim = 3;
      // max step length
      maxStepLength = 0.005; //0.05 m -> jak rozdzielczosc pojedynczego voxela

      // construct the state space we are planning in
      ob::StateSpacePtr space2;
      space2 = ob::StateSpacePtr(new ob::SE3StateSpace());

      // create a start state
      ob::ScopedState<ob::SE3StateSpace> start(space2);

      // create a goal state
      ob::ScopedState<ob::SE3StateSpace> goal(space2);

      // set the bounds for the R^3 part of SE(3)
      ob::RealVectorBounds bounds(dim); // 3 dim

      bounds.setLow(-1);
      bounds.setHigh(10);

      bounds.setLow(-10);
      bounds.setHigh(10);
//dziala tylko to -> kazdy wymiar -10, 10
      bounds.setLow(-10);
      bounds.setHigh(10);

      space2->as<ob::SE3StateSpace>()->setBounds(bounds);

      // construct an instance of  space information from this state space
      start->setXYZ(x_start,y_start,z_start);
      start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

      goal->setXYZ(x_docelowy,y_docelowy,z_docelowy);
      goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();


    // search space information
    auto si(std::make_shared<ompl::base::SpaceInformation>(space2));

    // define state checking callback

    si->setStateValidityChecker(isStateValid);
    // set State Validity Checking Resolution (avoid going through the walls)
    si->setStateValidityCheckingResolution(0.001); // 1 promil dokladnosci

    // problem definition
    auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));
    //state2
    pdef->setStartAndGoalStates(start, goal);

    // create and select planner

      //RRT - Connect
      auto planner(std::make_shared<ompl::geometric::RRTConnect>(si));

      //RRT*
      //auto planner(std::make_shared<og::RRTstar>(si));

      // zwykly RRT
      //auto planner(std::make_shared<og::RRT>(si));


      //Lazy PRM*
     //auto planner(std::make_shared<og::LazyPRMstar>(si));

     //auto planner(std::make_shared<og::PRM>(si));
     pdef->setOptimizationObjective(allocateObjective(si));
     //planner->setMaxNearestNeighbors(5);

    //parallel rrt
    // auto planner(std::make_shared<og::pRRT>(si));




  // configure the planner

     //uwaga -> dla PRM zakomentowac
  //planner->setRange(maxStepLength);// max step length

  //if RRT*

  //RRT* end

  planner->setProblemDefinition(pdef);
  planner->setRange(maxStepLength);// max step length // jeszcze raz

  //std::cout << "\npdef->print\n";
  pdef->print(std::cout);
  planner->setup();


  std::cout << "\nplanner->printSettings\n";
  planner->printSettings (std::cout);
  std::cout << "\npdef->print\n";
  pdef->print();


      // solve motion planning problem - max 10 sekund
      ompl::base::PlannerStatus solved = planner->ompl::base::Planner::solve(10.0);
      std::cout << "po planowaniu!\n";

      // publisher sciezki
      path_publisher = nh.advertise<nav_msgs::Path>( "paff", 0);
      path_nav_msgs.poses.clear(); //wyczysc poprzednie sciezki jesli jakies sa
      //jesli rozwiazano
      if (solved)
      {
          std::cout << "SOLVED\n\n\n";
          ob::PathPtr path = pdef->getSolutionPath();
          //path->print(std::cout); // print the path to screen
          //og::PathGeometric* pth = pdef->getSolutionPath()->as<og::PathGeometric>();
          const auto *pth = path.get()->as<og::PathGeometric>();
         //pth->printAsMatrix(std::cout);

          // do publikowania markerow
        for (std::size_t path_idx = 0; path_idx < pth->getStateCount(); path_idx++)
        {
            const ob::SE3StateSpace::StateType *se3state = pth->getState(path_idx)->as<ob::SE3StateSpace::StateType>();

            // extract the first component of the state and cast it to what we expect
            const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

            // extract the second component of the state and cast it to what we expect
            const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

            geometry_msgs::PoseStamped pose;

            pose.header.stamp = ros::Time();
            pose.header.frame_id = "world";
            pose.pose.position.x = pos->values[0];
            pose.pose.position.y = pos->values[1];
            pose.pose.position.z = pos->values[2];


            path_nav_msgs.header.stamp = ros::Time();
            path_nav_msgs.header.frame_id = "world";
            path_nav_msgs.poses.push_back(pose);

           // ros::Duration(0.1).sleep();

           /*
           std::cout << "Published marker: " << path_idx << std::endl;
           std::cout << "X = " << pos->values[0] << std::endl;
           std::cout << "Y = " << pos->values[1] << std::endl;
           std::cout << "Z = " << pos->values[2] << std::endl;
           */
           //zapisz dany punkt sciezki do ruchu dronem
           sciezka_x[path_idx] = pos->values[0];
           sciezka_y[path_idx] = pos->values[1];
           sciezka_z[path_idx] = pos->values[2];
           dlugosc_sciezki = path_idx; //zapisz dlugosc sciezki
        }
      }
      path_publisher.publish(path_nav_msgs); //publikuj sciezke.
  /*    PLANOWANIE RUCHU    */

      double dt = 0.05*2; //okresla co ile sie zapisuje do .bag
      double t = 1e-1;
      //std::cout << "min_x = " << min_x << std::endl;
      //std::cout << "min_y = " << min_y << std::endl;
      //std::cout << "min_z = " << min_z << std::endl;

      //najpierw nagrywa do .bag, pozniej to odtwarza
      std::cout << "Wizualizacja\n";
      for(int i=0;i<dlugosc_sciezki;i++)
      {
          //do poruszania dronem - kolejne probki sciezki
          base.lin.p_.x() = sciezka_x[i];
          base.lin.p_.y() = sciezka_y[i];
          base.lin.p_.z() = sciezka_z[i];

          // to mozna pominac -> rotacja jest nieistotna
          double roll = 0;
          base.ang.q = GetQuaternionFromEulerZYX(0.0, 0.0, roll);

          // save the state message with current time in the bag
          auto timestamp = ::ros::Time(t);
          xpp_msgs::RobotStateJoint msg;
          msg.base = Convert::ToRos(base);

          //zapisuj do .bag
          bag.write("xpp/joint_quadrotor_des", timestamp, msg);

          t += dt;
          //std::cout << "Sciezka: x = " << sciezka_x[i] << " y = " << sciezka_y[i] << " z = " << sciezka_z[i] << std::endl;

      }
      std::cout << "DLUGOSC SCIEZKI: " << dlugosc_sciezki << std::endl;

      std::string bag_file = bag.getFileName(); // save location before closing
      bag.close();

      // plays back the states at desired speeds.
      // can now also use all the tools from rosbag to pause, speed-up, inspect,
      // debug the trajectory.
      // e.g. rosbag -play -r 0.2 /path/to/bag.bag
      int success = system(("rosbag play " + bag_file).c_str());

      //do planowania kilka razy - wyswietl sciezke i poczekaj.
      int xxx = 0;
      while (xxx < 5)
      {
          xxx++;
          ros::Rate r(1);
          r.sleep();
      }

  } //planuje 5x - petla for koniec
  while( ros::ok())
  {
    path_publisher.publish(path_nav_msgs);
    ros::Rate r(10);
    r.sleep();
  }

  return 0;
}


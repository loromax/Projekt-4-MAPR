#include <ros/ros.h>

#include <xpp_vis/urdf_visualizer.h>

#include <xpp_msgs/RobotStateCartesian.h>
#include <xpp_msgs/topic_names.h>


ros::Publisher joint_state_pub;



void StateCallback(const xpp_msgs::RobotStateCartesian& msg)
{

  xpp_msgs::RobotStateJoint joint_msg;
  joint_msg.base = msg.base;
  joint_state_pub.publish(joint_msg);
}


int main(int argc, char *argv[])
{
    //inicjalizacja node'a
  ros::init(argc, argv, "quadrotor_urdf_visualizer");
  ros::NodeHandle n;


  const std::string joint_quadrotor = "xpp/joint_quadrotor_des";
  ros::Subscriber cart_state_sub = n.subscribe(xpp_msgs::robot_state_desired, 1, StateCallback);
  joint_state_pub = n.advertise<xpp_msgs::RobotStateJoint>(joint_quadrotor, 1);

  // publish base state to RVIZ
  std::string urdf = "quadrotor_rviz_urdf_robot_description";
  // zamien frejma drona z 'base' na 'world'
  xpp::UrdfVisualizer node_des(urdf, {}, "base", "world", joint_quadrotor, "quadrotor");

  ros::spin();

  return 0;
}


/*
 * braitenberg_node.cpp
 *
 *  Created on: 20.12.2015
 *      Author: chris
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>


sensor_msgs::LaserScan _scan;
ros::Publisher         _twistPub;

const double _phi_threshold   = 0.5;
const double _range_threshold = 1.0;

float isAngleInDrivingDirection(const double& phi)
{
   return 1.0;
}


float isDistanceGreat(const double& range)
{
   return 1.0;
}

geometry_msgs::Point getCartesianCoordinate(const double& range, const double phi)
{
   geometry_msgs::Point point;
   point.x = range*cos(phi);
   point.y = range*sin(phi);
   return point;
}

bool isInDrivingWay(const double& range, const double& phi)
{
   const geometry_msgs::Point p = getCartesianCoordinate(range, phi);
   if((::fabs(p.y) < 0.25)) return false;

   return true;
}




/**
 * Function to publish velocity
 */
void publishTwist(void)
{

   // please implement a reactive laser behaviour in the following lines
   double alpha = 0.0;  // speed for yaw angle
   double speed = 0.0;  // speed in x direction


  double sum_1    = 0.0;         // nominator of atan2 function and
  double sum_2    = 0.0;         // denominator of atan2 function used for orientation
  double min_dist = 5;           // variable to control speed

  for(unsigned int i=0 ; i<_scan.ranges.size() ; ++i)
  {


  }

   alpha = ::atan2(sum_1, sum_2);
//   speed = 1.0;
   speed = 0.0;


   geometry_msgs::Twist twist;
   twist.linear.x  = speed;
   twist.angular.z = alpha;





  /*
   * bug fix for simulation:
   * Values close to zero cause problems in the simulation.
   * Below a given threshold the velocity is set to zero to
   * solve this problem.
   */
   if (speed < 0.01)                 twist.angular.z = 0.1;
   if (fabs(twist.angular.z) < 0.05) twist.angular.z = 0.0;





   /* limit velocity for simulation
    * please do not change the following lines
    * this is necessary so the simulation keeps being close
    * to the real world
    */
   const float v_max   = 1.0; // define maximum velocity in x direction
   const float yaw_max = 1.0; // define maximum turn speed in rad

   if(twist.linear.x  > v_max )   twist.linear.x  =  v_max;
   if(twist.linear.x  < -v_max)   twist.linear.x  = -v_max;
   if(twist.angular.z > yaw_max)  twist.angular.z =  yaw_max;
   if(twist.angular.z < -yaw_max) twist.angular.z = -yaw_max;

   // debug output to console
   ROS_INFO_STREAM("linear  x: " << twist.linear.x);
   ROS_INFO_STREAM("angular z: " << twist.angular.z);

   // broadcast velocity
   _twistPub.publish(twist);
}

/**
 * Callback function for laser
 */
void laserCallback(const sensor_msgs::LaserScan &msg)
{
   _scan = msg;
   publishTwist();
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "reactive_laser");

   ros::NodeHandle nh;

   const std::string vel_topic;
   const std::string laser_topic;

   _twistPub = nh.advertise < geometry_msgs::Twist > ("robot0/cmd_vel", 1);
   ros::Subscriber rangeLeftSub = nh.subscribe("robot0/laser_0", 1, laserCallback);

   ros::spin();

}

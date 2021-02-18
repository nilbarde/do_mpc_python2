#include<iostream>
#include<fstream>
#include<stdlib.h>
#include<vector>
#include<time.h>
#include<math.h>

#include "ros/ros.h"
#include "std_msgs/Header.h"
#include <std_msgs/Float32.h>
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include <std_msgs/Float64MultiArray.h>

using namespace std;
float max_v = 0;
class SubscribeAndPublishV
{
public:
  SubscribeAndPublishV()
  {
    //Topic you want to publish
    v_pub = n.advertise<std_msgs::Float32>("v_max", 1);

    //Topic you want to subscribe
    sub = n.subscribe("/velocity_array", 1, &SubscribeAndPublishV::callback, this);
  }

  void callback(const std_msgs::Float64MultiArray::ConstPtr& x)
  {
   int len = x->data.size();
  float arr[len];
  for(int i=0; i<len; i++)
    {
      arr[i] = x->data[i];
    }
  
  for(int i=0; i<len; i++)
  {
    if(arr[i]>max_v){
      max_v = arr[i];
    }
  }
  //fstream fout;
  //fout.open("max_vel.csv",ios::out | ios::app);
  //fout<<max<<"\n";
  ROS_INFO("[%f]", max_v);
  std_msgs::Float32 msg_v;
  msg_v.data=max_v;
  v_pub.publish(msg_v);
  }

private:
  ros::NodeHandle n; 
  ros::Publisher v_pub;
  ros::Subscriber sub;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "vel_subscriber");

  //Create an object of class SubscribeAndPublishV that will take care of everything
  SubscribeAndPublishV SAPObject;

  ros::spin();

  return 0;
}
/*void Callback(const std_msgs::Float64MultiArray::ConstPtr& x)
{
  int len = x->data.size();
  double arr[len];
  for(int i=0; i<len; i++)
    {
      arr[i] = x->data[i];
    }
  double max = 0;
  for(int i=0; i<len; i++)
  {
    if(arr[i]>max){
      max = arr[i];
    }
  }
  //fstream fout;
  //fout.open("max_vel.csv",ios::out | ios::app);
  //fout<<max<<"\n";
  ROS_INFO("[%f]", max);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vel_subscriber");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/velocity_array", 1, Callback);

  ros::spin();

  return 0;
}*/

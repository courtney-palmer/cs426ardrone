#include <math.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "std_msgs/Float32.h"

class WorldViewInterpreter{

  std::vector<geometry_msgs::Point32> pointCloud;
  sensor_msgs::PointCloud rosPointCloud;
  ros::NodeHandle nodeHandle;
  ros::Subscriber laserSub;
  ros::Subscriber lidarOrientationSub;
  ros::Publisher pointCloudPub;
  ros::Rate loop_rate = ros::Rate(30);

  int rotY = 0;
  float orientation = 0;

  public:

  WorldViewInterpreter(){
    laserSub = nodeHandle.subscribe("/laser_publisher/laser_scan", 100, &WorldViewInterpreter::laserToPoint, this);

    lidarOrientationSub = nodeHandle.subscribe("/laser_publisher/lidar_orientation", 100, &WorldViewInterpreter::onRotation, this);

    pointCloudPub = nodeHandle.advertise<sensor_msgs::PointCloud>("/point_cloud", 1);
  }

  void laserToPoint(const sensor_msgs::LaserScan::ConstPtr& msg){

    int numLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    std::cout << "numLasers: " << numLasers << std::endl;
    geometry_msgs::Point32 point;
    float distance;
    float horizontalAngle;
    float verticalAngle;

    for(int i = 0; i < 64; i++){
      distance = msg->ranges[i];
      horizontalAngle = this->orientation; //todo
      verticalAngle= (1.5708 - (msg->angle_min + (i * msg->angle_increment)) + 0.785398);

      point.x = getX(distance, horizontalAngle, verticalAngle);
      std::cout << distance << std::endl;
      point.y = getY(distance, horizontalAngle, verticalAngle);
      point.z = -getZ(distance, verticalAngle);

      this->pointCloud.push_back(point);
    }
     rosPointCloud.points.resize(pointCloud.size());
     std::copy(pointCloud.begin(), pointCloud.end(), std::back_inserter(rosPointCloud.points));

     rosPointCloud.header.frame_id = "my_frame";

     pointCloudPub.publish(rosPointCloud);
     loop_rate.sleep();

     rotY++;
  }

  void onRotation(const std_msgs::Float32ConstPtr& msg){
    this->orientation = msg->data;
  }

  float getX(float distance, float horizontalAngle, float verticalAngle){
    return distance * sin(verticalAngle) * cos(horizontalAngle); 
  }

  float getY(float distance, float horizontalAngle, float verticalAngle){
    return distance * sin(verticalAngle) * sin(horizontalAngle);
  }

  float getZ(float distance, float verticalAngle){
    return distance * cos(verticalAngle);
  }
  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_listener");
  WorldViewInterpreter interpreter;
  ros::spin();
  return 0;
}

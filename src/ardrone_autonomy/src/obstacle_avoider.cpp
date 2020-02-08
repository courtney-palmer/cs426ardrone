
class ObstacleAvoider{

  public:

  ObstacleAvoider() {}

  private:

  ros::Subscriber laserSub;
  ros::Subscriber imageSub;

  ros::Publisher velPub;

};

int main(int argc, char** argv){

  ros::init(argc, argv, "obstacle_avoider");
  ObstacleAvoider obstacleAvoider;
  ros::spin();
  return 0;
}

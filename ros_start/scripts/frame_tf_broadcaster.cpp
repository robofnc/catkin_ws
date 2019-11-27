#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>

#define WX 0.0999593
#define WY 0.164083
#define WZ 1.08738
#define IX 0.204787
#define IY 0.447486
#define IZ 0.726125

/*
void inp(const geometry_msgs::Point::ConstPtr& point)
{
    ROS_INFO("I heard 1");
}
void wayp(const geometry_msgs::Point::ConstPtr& point)
{
    ROS_INFO("I heard 2");
}
*/

class getpcd {
private:
  ros::NodeHandle node;
  ros::Subscriber kinepoint;
  ros::Publisher ros_point; 
  //int save_count;
public:
  getpcd(){
    //* subscribe ROS topics
    kinepoint = node.subscribe ("/point", 1,  &getpcd::point_read, this);
  }
  ~getpcd() {}
  //* get points


void bemain(geometry_msgs::Twist point)
{
  ros::NodeHandle node;
  geometry_msgs::Point in,way;

  //double in[3],way[3];
  //ros::Subscriber sub1 = node.subscribe("inpoint", 1000, inp);
  //ros::Subscriber sub2 = node.subscribe("waypoint", 1000, wayp);

  
  double roll = 0.0*M_PI/180.0, pitch = 27.5*M_PI/180.0, yaw = 0.0*M_PI/180.0;

  geometry_msgs::Quaternion s;
  geometry_msgs::Quaternion &q = s;
  
  tf::Quaternion quat=tf::createQuaternionFromRPY(roll,pitch,yaw);
  quaternionTFToMsg(quat,q);


  double roll2 = 0.0*M_PI/180.0, pitch2 = 90.0*M_PI/180.0, yaw2 = 0.0*M_PI/180.0;

  geometry_msgs::Quaternion s2;
  geometry_msgs::Quaternion &q2 = s2;
  
  tf::Quaternion quat2=tf::createQuaternionFromRPY(roll2,pitch2,yaw2);
  quaternionTFToMsg(quat2,q2);

  std::cout << "x = " << q.x << std::endl;
  std::cout << "y = " << q.y << std::endl;
  std::cout << "z = " << q.z << std::endl;
  std::cout << "w = " << q.w << std::endl;
  
  tf::TransformBroadcaster br;
  tf::TransformBroadcaster br2;
  tf::TransformBroadcaster br3;
  
  tf::Transform transform;
  tf::Transform transform2;
  tf::Transform transform3;

  //double wx = 1.15279, wy = 0.0168866,wz = 0.00957468,tt,ww;
  //double wx = 0.8, wy = -0.18,wz = -0.52,tt,ww;


  //double wx = WZ, wy = -WX,wz = -WY,tt,ww; //手入力用
  double wx = point.linear.z, wy = -point.linear.x,wz = -point.linear.y,tt,ww;

  //double ix = 0.979455, iy = 0.117589, iz = 0.233083, x, y, z;
  //double ix = 1.06, iy = -0.078, iz = 0.0017, x, y, z,ei;


  //double ix = IZ,iy = -IX, iz = -IY, x, y, z,ei; //手入力用
  double ix = point.angular.z,iy = -point.angular.x, iz = -point.angular.y, x, y, z,ei;
  
  //x = ix - wx;
  //y = iy - wy;
  //z = iz - wz;
  x = wx - ix;
  y = wy - iy;
  z = wz - iz;
  ei = sqrt(y*y + z*z);
  tt = sqrt(( y*y + z*z) / ( x*x + y*y + z*z));
  ww = asin(tt);
  tf::TransformListener tflistener;

  
  //geometry_msgs::Vector3Stamped point;
  //geometry_msgs::Vector3Stamped basepoint;
  //point.vector.x = 0.1;
  //point.vector.y = 0.5;
  //point.vector.z = 0.8;
  //point.header.stamp = ros::Time::now();

  //basepoint.header = point.header;
 

  ros::Rate rate(10.0);
  while (node.ok()){
    transform.setOrigin( tf::Vector3(0.05, 0.04, 0.14) );
    //transform.setOrigin( tf::Vector3(0.5, 0.5, -0.2) );
    transform.setRotation( tf::Quaternion(q.x, q.y, q.z, q.w) );
    //transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "sonar_ring", "camera"));
    //tflistener.lookupTransform("/base", "/carrot1", ros::Time(0), transform);
    //sleep(2.0);
    //transform2.setOrigin( tf::Vector3(1.0, 0.0, 0.0) );
    transform2.setOrigin( tf::Vector3(wx, wy, wz) );
    transform2.setRotation( tf::Quaternion(0, -z*sin(ww/2.0)/ei, y*sin(ww/2.0)/ei, cos(ww/2.0)) );
    //transform2.setRotation( tf::Quaternion(0, 0.7, 0, 0.7));
    br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "camera", "hole"));

    transform3.setOrigin( tf::Vector3(ix, iy, iz) );
    //transform3.setRotation( tf::Quaternion(0, -z, y, ww) );
    transform3.setRotation( tf::Quaternion(0, -z*sin(ww/2.0)/ei, y*sin(ww/2.0)/ei, cos(ww/2.0)) );
    //transform3.setRotation( tf::Quaternion(q2.x, q2.y, q2.z, q2.w) );
    br3.sendTransform(tf::StampedTransform(transform3, ros::Time::now(), "camera", "ppp"));
    rate.sleep();
  }
  
  //tflistener.transformVector("carrot1",point.header.stamp,point,"base",basepoint);
  //basepoint.vector.x = 
  //std::cout << "basex = " << basepoint.vector.x << std::endl;
  //std::cout << "basey = " << basepoint.vector.y << std::endl;
  //std::cout << "basez = " << basepoint.vector.z << std::endl;
}

void point_read( const geometry_msgs::Twist point){

    geometry_msgs::Twist pp;
    pp.linear.x = point.linear.x;
    pp.linear.y = point.linear.y;
    pp.linear.z = point.linear.z;
    pp.angular.x = point.angular.x;
    pp.angular.y = point.angular.y;
    pp.angular.z = point.angular.z;
    bemain(pp);
    usleep( 300 );
  }

};

int main( int argc, char** argv ){
  ros::init(argc,argv,"my_tf_broadcaster");
  getpcd spcd;
  ros::spin();

  return 1;
}




#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

static ros::Publisher  s_pub;

struct PointXYZRI
{
  PCL_ADD_POINT4D;
  float intensity;
  int ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZRI,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (uint16_t, ring, ring))

void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::PointCloud<PointXYZRI>::Ptr cloud_xyzri(new pcl::PointCloud<PointXYZRI>);
  pcl::fromROSMsg(*msg, *cloud_xyzri);

  // Add extra field "ring" to PointCloud2
  sensor_msgs::PointCloud2 cloud_ros;
  for(int i=0; i< cloud_xyzri->points.size(); ++i){
      cloud_xyzri->points[i].ring = 0;
  }
  std::cout << "pcd " << cloud_xyzri->width << " " << cloud_xyzri->height << std::endl;
  cloud_xyzri->is_dense = true;
  pcl::toROSMsg(*cloud_xyzri, cloud_ros);

  cloud_ros.header = msg->header;
  // Do something with cloud_ros here

  s_pub.publish(cloud_ros);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "add_ring_to_pointcloud2");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("input_pointcloud", 10, callback);
  s_pub = nh.advertise<sensor_msgs::PointCloud2>("points_raw", 10);

  ros::spin();

  return 0;
}


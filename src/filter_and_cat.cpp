// #include "../include/lidar_calibration/filtercat.h"
#include "lidar_calibration/calibration.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

pcl::PointCloud<pcl::PointXYZI>::Ptr high_pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr low_pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>());
using namespace message_filters;

lidar_calibration::lidar_calibration(ros::NodeHandle &_nh)
{
    nh = _nh;

}

void callback(const sensor_msgs::PointCloud2ConstPtr &high_pc, const sensor_msgs::PointCloud2ConstPtr &low_pc)
{
  // Solve all of perception here...
  printf("------high lidar receive stamp---------\n");
  printf("secs %f, neces %f", high_pc->header.stamp);
  printf("------low lidar receive stamp---------\n");
  printf("secs %f, neces %f", low_pc->header.stamp);

  ROS_INFO("cat start");

  pcl::fromROSMsg(*high_pc, *high_pcl_cloud);
  pcl::fromROSMsg(*low_pc, *low_pcl_cloud);

  pcl::PointCloud<pcl::PointXYZI> cloud_all = *high_pcl_cloud + *low_pcl_cloud;  //拼接点云
  
  ROS_INFO("cat done");


//   pcl::PCLPointCloud2 pcl_pointcloud;
//   pcl::toPCLPointCloud2(*cloud, pcl_pointcloud);
  string nsec = to_string(low_pc->header.stamp.nsec);
  string sec = to_string(low_pc->header.stamp.sec);
  string name = sec + nsec + ".pcd";
  pcl::io::savePCDFile (name, cloud_all);
  ROS_INFO("save done");

//   sensor_msgs::PointCloud2 allmsg;
//   pcl::toROSMsg(cloud_all, msg);
//   allmsg.header.frame_id = "rslidar";
//   allmsg.header.stamp = ros::Time::now();
//   pub_all.publish(allmsg);


}

int main(int argc, char *argv[])
{
    printf("node init\n");
    ros::init(argc, argv, "filt_and_cat_nocd");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::PointCloud2> low_sub(nh, "/pole_low/rslidar_points", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> high_sub(nh, "/pole_high/rslidar_points", 1);

    lidar_calibration L(nh);

    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2ConstPtr, sensor_msgs::PointCloud2ConstPtr> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), low_sub, high_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));
 
    ros::spin();
    return 0;
}
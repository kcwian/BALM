#include "tools.hpp"
#include <ros/ros.h>
#include <Eigen/Eigenvalues>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseArray.h>
#include <random>
#include <ctime>
#include <tf/transform_broadcaster.h>
#include "bavoxel.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <malloc.h>
#include <Eigen/Core>
#include <fstream>

using namespace std;

template <typename T>
void pub_pl_func(T &pl, ros::Publisher &pub)
{
  pl.height = 1; pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "camera_init";
  output.header.stamp = ros::Time::now();
  pub.publish(output);
}

ros::Publisher pub_path, pub_test, pub_show, pub_cute;


#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>

// Define the PointCloud type (for example pcl::PointXYZ)
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// int read_data(vector<double> &tims, PLM(3) &rots, PLV(3) &poss, string prename)
int read_data(vector<IMUST> &x_buf, vector<pcl::PointCloud<PointType>::Ptr> &pl_fulls, string &bag_filepath, string & odom_topic, string & lidar_topic, int poses_limit)
{
    std::string pointcloud_topic = lidar_topic;  // Change to your actual PointCloud2 topic name
    std::string odometry_topic = odom_topic;      // Change to your actual Odometry topic name
    int pose_size = 0;
    int cloud_size = 0;
    // Open the rosbag file
    rosbag::Bag bag;
    try {
        bag.open(bag_filepath, rosbag::bagmode::Read);
    } catch (rosbag::BagException &e) {
        ROS_ERROR("Error opening bag file: %s", e.what());
        return -1;
    }

    // Specify the topics you want to read
    std::vector<std::string> topics;
    topics.push_back(odometry_topic);
    rosbag::View view_odom(bag, rosbag::TopicQuery(topics));
    int odom_msgs = 0;
    // Iterate over the messages in the bag file
    for (rosbag::View::iterator it = view_odom.begin(); it != view_odom.end(); ++it) {
            odom_msgs++;
    }
    std::cout << "Odom_msgs: " << odom_msgs << std::endl;
    topics.push_back(pointcloud_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    // Iterate through the bag file
    int decimation = (float) std::ceil(odom_msgs/poses_limit) / voxel_size;
    std::cout << "Decimation: " << decimation << std::endl;
    for (const rosbag::MessageInstance &msg : view) {
        // Check if the message is a PointCloud2
        if (msg.getTopic() == pointcloud_topic || msg.isType<sensor_msgs::PointCloud2>()) {
            sensor_msgs::PointCloud2::ConstPtr pc2_msg = msg.instantiate<sensor_msgs::PointCloud2>();
            if (pc2_msg != nullptr) {
                static int cloud_cnt = 0;
                if (cloud_cnt++ % decimation == 0){
                // Convert the sensor_msgs/PointCloud2 message to a PCL PointCloud
                pcl::PointCloud<pcl::PointXYZI> pl_tem;
                pcl::fromROSMsg(*pc2_msg, pl_tem);
                pcl::PointCloud<PointType>::Ptr pl_ptr(new pcl::PointCloud<PointType>());
                for (pcl::PointXYZI &pp : pl_tem.points) {
                  PointType ap;
                  ap.x = pp.x;
                  ap.y = pp.y;
                  ap.z = pp.z;
                  ap.intensity = pp.intensity;
                  pl_ptr->push_back(ap);
                }
                cloud_size++;
                pl_fulls.push_back(pl_ptr);
		}
            }
        }

        // Check if the message is an Odometry message
        if (msg.getTopic() == odometry_topic || msg.isType<nav_msgs::Odometry>()) {
            nav_msgs::Odometry::ConstPtr odom_msg = msg.instantiate<nav_msgs::Odometry>();
            if (odom_msg != nullptr) {
	      static int odom_cnt = 0;
              if (odom_cnt++ % decimation == 0){
              // Process the odometry data
              // ROS_INFO("Odometry received: Position (x: %f, y: %f, z: %f), Orientation (x: %f, y: %f, z: %f, w: %f)",
              double x = odom_msg->pose.pose.position.x;
              double y = odom_msg->pose.pose.position.y;
              double z = odom_msg->pose.pose.position.z;
              double qx = odom_msg->pose.pose.orientation.x;
              double qy = odom_msg->pose.pose.orientation.y;
              double qz = odom_msg->pose.pose.orientation.z;
              double qw = odom_msg->pose.pose.orientation.w;

              Eigen::Affine3d aff = Eigen::Affine3d::Identity();
              aff.translation() = Eigen::Vector3d(x, y, z);
              aff.linear() = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();

              PLV(3) poss;
              PLM(3) rots;
              vector<double> tims;
              rots.push_back(aff.matrix().block<3, 3>(0, 0));
              poss.push_back(aff.matrix().block<3, 1>(0, 3));
              tims.push_back(odom_msg->header.stamp.toSec());
              pose_size++;

              IMUST curr;
              curr.R = rots[0];
              curr.p = poss[0];
              curr.t = tims[0];
              x_buf.push_back(curr);
                }
	     }
        }
    }

    // Close the bag file
    bag.close();
    std::cout << "Poses: " << pose_size << " Scans: " << cloud_size << std::endl;
    return pose_size;
}


int read_pose(vector<double> &tims, PLM(3) &rots, PLV(3) &poss, string prename)
{
  string readname = prename + "alidarPose.csv";

  cout << readname << endl;
  ifstream inFile(readname);

  if(!inFile.is_open())
  {
    printf("open fail\n"); return 0;
  }

  int pose_size = 0;
  string lineStr, str;
  Eigen::Matrix4d aff;
  vector<double> nums;

  int ord = 0;
  while(getline(inFile, lineStr))
  {
    ord++;
    stringstream ss(lineStr);
    while(getline(ss, str, ','))
      nums.push_back(stod(str));

    if(ord == 4)
    {
      for(int j=0; j<16; j++)
        aff(j) = nums[j];

      Eigen::Matrix4d affT = aff.transpose();

      rots.push_back(affT.block<3, 3>(0, 0));
      poss.push_back(affT.block<3, 1>(0, 3));
      tims.push_back(affT(3, 3));
      nums.clear();
      ord = 0;
      pose_size++;
    }
  }

  return pose_size;
}

void read_file(vector<IMUST> &x_buf, vector<pcl::PointCloud<PointType>::Ptr> &pl_fulls, string &prename)
{
  prename = prename + "/datas/benchmark_realworld/";

  PLV(3) poss; PLM(3) rots;
  vector<double> tims;
  int pose_size = read_pose(tims, rots, poss, prename);
  
  for(int m=0; m<pose_size; m++)
  {
    string filename = prename + "full" + to_string(m) + ".pcd";

    pcl::PointCloud<PointType>::Ptr pl_ptr(new pcl::PointCloud<PointType>());
    pcl::PointCloud<pcl::PointXYZI> pl_tem;
    pcl::io::loadPCDFile(filename, pl_tem);
    for(pcl::PointXYZI &pp: pl_tem.points)
    {
      PointType ap;
      ap.x = pp.x; ap.y = pp.y; ap.z = pp.z;
      ap.intensity = pp.intensity;
      pl_ptr->push_back(ap);
    }

    pl_fulls.push_back(pl_ptr);

    IMUST curr;
    curr.R = rots[m]; curr.p = poss[m]; curr.t = tims[m];
    x_buf.push_back(curr);
  }
  

}

void data_show(vector<IMUST> x_buf, vector<pcl::PointCloud<PointType>::Ptr> &pl_fulls)
{
  IMUST es0 = x_buf[0];
  for(uint i=0; i<x_buf.size(); i++)
  {
    x_buf[i].p = es0.R.transpose() * (x_buf[i].p - es0.p);
    x_buf[i].R = es0.R.transpose() * x_buf[i].R;
  }

  pcl::PointCloud<PointType> pl_send, pl_path;
  int winsize = x_buf.size();
  for(int i=0; i<winsize; i++)
  {
    pcl::PointCloud<PointType> pl_tem = *pl_fulls[i];
    down_sampling_voxel(pl_tem, 0.05);
    pl_transform(pl_tem, x_buf[i]);
    pl_send += pl_tem;

    if((i%200==0 && i!=0) || i == winsize-1)
    {
      pub_pl_func(pl_send, pub_show);
      pl_send.clear();
      sleep(0.5);
    }

    PointType ap;
    ap.x = x_buf[i].p.x();
    ap.y = x_buf[i].p.y();
    ap.z = x_buf[i].p.z();
    ap.curvature = i;
    pl_path.push_back(ap);
  }

  pub_pl_func(pl_path, pub_path);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "benchmark2");
  ros::NodeHandle n;
  pub_test = n.advertise<sensor_msgs::PointCloud2>("/map_test", 100);
  pub_path = n.advertise<sensor_msgs::PointCloud2>("/map_path", 100);
  pub_show = n.advertise<sensor_msgs::PointCloud2>("/map_show", 100);
  pub_cute = n.advertise<sensor_msgs::PointCloud2>("/map_cute", 100);


    string prename, ofname;
    vector<IMUST> x_buf;
    vector<pcl::PointCloud<PointType>::Ptr> pl_fulls;

    n.param<double>("voxel_size", voxel_size, 1);
    std::cout << "Voxel size: " << voxel_size << std::endl;
    string file_path, balm_trajectory, odom_topic, lidar_topic;
    int poses_limit;
    n.param<string>("file_path", file_path, "");
    n.param<string>("trajectory_output_path", balm_trajectory, "");
    n.param<string>("odom_topic", odom_topic, "");
    n.param<string>("lidar_topic", lidar_topic, "");
    n.param<int>("poses_limit", poses_limit, 150);

    read_data(x_buf, pl_fulls, file_path, odom_topic, lidar_topic, poses_limit);
    //   read_file(x_buf, pl_fulls, file_path);

    IMUST es0 = x_buf[0];
    for (uint i = 0; i < x_buf.size(); i++) {
      x_buf[i].p = es0.R.transpose() * (x_buf[i].p - es0.p);
      x_buf[i].R = es0.R.transpose() * x_buf[i].R;
    }

    win_size = x_buf.size();
    printf("The size of poses: %d\n", win_size);

    // data_show(x_buf, pl_fulls);
    printf("Check the point cloud with the initial poses.\n");
    printf("If no problem, input '1' to continue or '0' to exit...\n");
    //  int a; cin >> a; if(a==0) exit(0);

    pcl::PointCloud<PointType> pl_full, pl_surf, pl_path, pl_send;
    for (int iterCount = 0; iterCount < 1; iterCount++) {
      unordered_map<VOXEL_LOC, OCTO_TREE_ROOT *> surf_map;

      eigen_value_array[0] = 1.0 / 16;
      eigen_value_array[1] = 1.0 / 16;
      eigen_value_array[2] = 1.0 / 9;

      for (int i = 0; i < win_size; i++)
        cut_voxel(surf_map, *pl_fulls[i], x_buf[i], i);

      pcl::PointCloud<PointType> pl_send;
      pub_pl_func(pl_send, pub_show);

      pcl::PointCloud<PointType> pl_cent;
      pl_send.clear();
      VOX_HESS voxhess;
      for (auto iter = surf_map.begin(); iter != surf_map.end() && n.ok(); iter++) {
        iter->second->recut(win_size);
        iter->second->tras_opt(voxhess, win_size);
        iter->second->tras_display(pl_send, win_size);
      }

      pub_pl_func(pl_send, pub_cute);
      printf("\nThe planes (point association) cut by adaptive voxelization.\n");
      printf("If the planes are too few, the optimization will be degenerated and fail.\n");
      printf("If no problem, input '1' to continue or '0' to exit...\n");
      //   int a; cin >> a; if(a==0) exit(0);
      pl_send.clear();
      pub_pl_func(pl_send, pub_cute);

      if (voxhess.plvec_voxels.size() < 3 * x_buf.size()) {
        printf("Initial error too large.\n");
        printf("Please loose plane determination criteria for more planes.\n");
        printf("The optimization is terminated.\n");
        //      exit(0);
      }
      BALM2 opt_lsv;

      opt_lsv.damping_iter(x_buf, voxhess);

      for (auto iter = surf_map.begin(); iter != surf_map.end();) {
        delete iter->second;
        surf_map.erase(iter++);
      }
      surf_map.clear();

      malloc_trim(0);
   

    printf("\nRefined point cloud is publishing...\n");
    malloc_trim(0);
    //  data_show(x_buf, pl_fulls);
    printf("\nRefined point cloud is published.\n");

    // dump output to txt file
    ofstream os(balm_trajectory + std::to_string((voxel_size)) + "_dec_" + std::to_string((10 / voxel_size)));
    os << fixed << setprecision(6);
    for (uint i = 0; i < x_buf.size(); i++) {
      Eigen::Quaterniond quat(x_buf[i].R);
      os << x_buf[i].t << " "
         << x_buf[i].p[0] << " "
         << x_buf[i].p[1] << " "
         << x_buf[i].p[2] << " "
         << quat.x() << " "
         << quat.y() << " "
         << quat.z() << " "
         << quat.w() << "\n";
    }
    cout << "trajectory output written in " << balm_trajectory << endl;
    os.close();
  }
//  ros::spin();
  return 0;

}

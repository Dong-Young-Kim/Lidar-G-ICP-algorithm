#include <ros/ros.h>
#include <boost/format.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/gicp.h>

#include <pcl/filters/voxel_grid.h> //voxelgird

ros::Publisher pub_res;


bool first_frame = true;
pcl::PointCloud<pcl::PointXYZI> src;
pcl::PointCloud<pcl::PointXYZI> tgt;

using namespace std;

void colorize(const pcl::PointCloud<pcl::PointXYZI> &pc, pcl::PointCloud<pcl::PointXYZRGB> &pc_colored, const vector<int> &color) {
    int N = pc.points.size();

    pc_colored.clear();
    pcl::PointXYZRGB pt_tmp;
    for (int i = 0; i < N; ++i) {
        const auto &pt = pc.points[i];
        pt_tmp.x = pt.x;
        pt_tmp.y = pt.y;
        pt_tmp.z = pt.z;
        pt_tmp.r = color[0];
        pt_tmp.g = color[1];
        pt_tmp.b = color[2];
        pc_colored.points.emplace_back(pt_tmp);
    }
}

void g_icp(){
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;
    gicp.setMaxCorrespondenceDistance(1.0);
    gicp.setTransformationEpsilon(0.003);
    gicp.setMaximumIterations(1000);

    pcl::PointCloud<pcl::PointXYZI>::Ptr align(new pcl::PointCloud<pcl::PointXYZI>);
    gicp.setInputSource(src.makeShared()); //make shared return heap/smart pointer
    gicp.setInputTarget(tgt.makeShared());
    gicp.align(*align);

    // Set outputs
    Eigen::Matrix4f src2tgt   = gicp.getFinalTransformation();
    double score     = gicp.getFitnessScore();
    bool is_converged = gicp.hasConverged();

    cout << "source to target matrix\n" << src2tgt << endl;
    cout << "matched score " << score << endl << endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr align_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    colorize(src, *src_colored, {255, 0, 0});
    colorize(tgt, *tgt_colored, {0, 255, 0});
    colorize(*align, *align_colored, {255, 0, 255});

    pcl::PointCloud<pcl::PointXYZRGB> pubCloud;
    pubCloud = *tgt_colored + *align_colored;

    sensor_msgs::PointCloud2 output;
    pcl::PCLPointCloud2 tmp_PCL;                               //declare PCL_PC2
    pcl::toPCLPointCloud2(pubCloud, tmp_PCL);                     //PC -> PCL_PC2
    pcl_conversions::fromPCL(tmp_PCL, output);                 //PCL_PC2 -> sensor_msg_PC2
    output.header.stamp = ros::Time::now();
    output.header.frame_id = "map";

    pub_res.publish(output);
}

void data_process(const sensor_msgs::PointCloud2ConstPtr& scan){
    pcl::PointCloud<pcl::PointXYZI> input_raw;
    pcl::PointCloud<pcl::PointXYZI> downsampled_cloud;
    pcl::fromROSMsg(*scan, input_raw);

    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(input_raw.makeShared());
    vg.setLeafSize(0.08f, 0.08f, 0.08f);
    vg.filter(downsampled_cloud);

    if(first_frame) {
        tgt = downsampled_cloud;
        first_frame = false;
        return;
    }

    //not first frame
    src = downsampled_cloud;
    g_icp();
    //prev_src = crnt_tgt;

}

int main(int argc, char**argv) {
    ros::init(argc, argv, "g_icp");             //node name 
	ros::NodeHandle nh;      
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 1, data_process);
    pub_res = nh.advertise<sensor_msgs::PointCloud2> ("/merged_cloud", 1);
    

    ros::spin();

}
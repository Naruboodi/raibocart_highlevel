#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>

#include <iostream>

class cloud_processing{

    private:

        ros::NodeHandle nh;

        ros::Subscriber velodyne_points_sub;
        ros::Subscriber realsense_depth_sub;

        ros::Publisher combind_cloud_pub;

        pcl::PointCloud<pcl::PointXYZ>::Ptr velodyne_temp{new pcl::PointCloud<pcl::PointXYZ>};
        pcl::PointCloud<pcl::PointXYZ>::Ptr realsense_temp{new pcl::PointCloud<pcl::PointXYZ>};

        pcl::PointCloud<pcl::PointXYZ> velodyne_pcl;
        pcl::PointCloud<pcl::PointXYZ> realsense_pcl;

        pcl::PointCloud<pcl::PointXYZ> combind_cloud_pcl;
        sensor_msgs::PointCloud2 combind_cloud_ros;

        pcl::PointCloud<pcl::PointXYZ> pc_transformed;
        pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_transformed{new pcl::PointCloud<pcl::PointXYZ>};

        std_msgs::Header velodyne_header;
        std_msgs::Header realsense_header;
        std_msgs::Header velodyne_header_temp;
        std_msgs::Header realsense_header_temp;

        bool velodyne_input = false;
        bool realsense_input = false;
        bool cloud_convert = false;

    public:

        cloud_processing():
            nh("~"){
                velodyne_points_sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points",1,&cloud_processing::call_back_handler,this);
                realsense_depth_sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points",1,&cloud_processing::call_back_handler,this);

                combind_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/combind_point_cloud", 1);
            }

    void call_back_handler(const sensor_msgs::PointCloud2ConstPtr& cloud_in){

        std_msgs::Header cloud_header = cloud_in->header;
        pcl::PCLPointCloud2 pcl_pc2;

        pcl_conversions::toPCL(*cloud_in,pcl_pc2);

        if(!velodyne_input && !realsense_input){
            velodyne_pcl.clear();
            realsense_pcl.clear();
            combind_cloud_pcl.clear();
            pc_transformed.clear();
            velodyne_temp->clear();
            realsense_temp->clear();
            ptr_transformed->clear();
        }
        
        if(cloud_header.frame_id == "velodyne"){
            
            if(!velodyne_input){
            velodyne_header = cloud_header;
            pcl::fromPCLPointCloud2(pcl_pc2,*velodyne_temp);
            velodyne_pcl = *velodyne_temp;
            velodyne_input = true;
            }
        }

        else if(cloud_header.frame_id == "camera_color_optical_frame"){
            if(!realsense_input){
            realsense_header = cloud_header;
            pcl::fromPCLPointCloud2(pcl_pc2,*realsense_temp);
            realsense_pcl = *realsense_temp;
            cloud_traslator();
            realsense_input = true;
            }
        }

        cloud_combinator();

    }

    void cloud_traslator(){


        Eigen::Matrix4f trans;
        trans<< 0,   0,  1, 0.36,
                -1,   0,  0, 0,
                0,   -1,  0, -1,
                0,   0,  0, 1;

        pcl::transformPointCloud(realsense_pcl, *ptr_transformed, trans);
        pc_transformed = *ptr_transformed;

    }

    void cloud_combinator(){

       if(realsense_input && velodyne_input){

            combind_cloud_pcl += velodyne_pcl;
            combind_cloud_pcl += pc_transformed;

            pcl::toROSMsg(combind_cloud_pcl, combind_cloud_ros);
            combind_cloud_ros.header.frame_id = "velodyne_realsense_trans";
            combind_cloud_pub.publish(combind_cloud_ros);

            velodyne_input = false;
            realsense_input = false;

        }

    }

};

int main(int argc, char** argv){

    ros::init(argc, argv, "cloud_processing");
    
    cloud_processing C_P;

    ros::Rate rate(100);

    while (ros::ok()) {
        ros::spinOnce(); 
        rate.sleep();
    }

    return 0;
}
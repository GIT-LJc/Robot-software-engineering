/*
此代码只用于寻找某个区域内的平均坐标
接受roi和pcd信息
*/
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <sstream>
#include <string>
#include <vector>
#include <unistd.h>
#include <cmath>

#include <boost/thread/thread.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

//ROS headers
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/RegionOfInterest.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_frame (new pcl::PointCloud<pcl::PointXYZ>);
using namespace std;
using namespace cv;
using namespace pcl;

class Depth{
private:
    ros::NodeHandle nh;
    int camera_width;
    int camera_height;
    // 计算数据
    unsigned char floatBuffer[4];
    // 视野数据
    sensor_msgs::RegionOfInterest posinview;
    // 工作状态
    bool work = false;

    // 接受和发布
    ros::Subscriber pointcloud_sub;
    ros::Subscriber posview_sub;

    ros::Publisher position_pub;

    int find_near_valid(int idx)
    {   
        double temp_min = 9999999999;
        int return_idx = idx;
        int obj_row = idx/camera_width;
        int obj_col = idx%camera_width;

        for (int row=0; row<camera_height; row++)
        {
            for (int col=0; col<camera_width; col++)
            {
                if (!isnan(cloud_frame->points[row*camera_width+col].x) &&
                    !isnan(cloud_frame->points[row*camera_width+col].y) &&
                    !isnan(cloud_frame->points[row*camera_width+col].z))
                {
                    double dis = (row-obj_row)*(row-obj_row) + (col-obj_col)*(col-obj_col);
                    if (dis < temp_min)
                    {
                        return_idx = row*camera_width + col;
                        temp_min = dis;
                    }
                }
            }
        }
        return return_idx;
    }
    // 点云回调函数
    void pclCallback (const sensor_msgs::PointCloud2& msg){
        if(work){
            work = false;
            double X_pos = 0.0;
            double Y_pos = 0.0;
            double Z_pos = 0.0;
            int times = 0;
            // 先Y再X
            pcl::fromROSMsg(msg, *cloud_frame);
            cout<<"The size is "<<posinview.height <<"\t"<<posinview.width<<endl;

            for(int i = posinview.y_offset; i<posinview.height+posinview.y_offset; i++){
                for(int j = posinview.x_offset; j<posinview.width+posinview.x_offset; j++){
                    int origin_index = i * camera_width + j;
                    if (isnan(cloud_frame->points[origin_index].x) || 
                        isnan(cloud_frame->points[origin_index].y) ||
                        isnan(cloud_frame->points[origin_index].z))
                            continue;

                    X_pos += cloud_frame->points[origin_index].x;
                    Y_pos += cloud_frame->points[origin_index].y;
                    Z_pos += cloud_frame->points[origin_index].z;
                    times += 1;
                }
            }

            geometry_msgs::PointStamped cam_pos;
            geometry_msgs::PointStamped base_pos;
            geometry_msgs::PointStamped map_pos;
            cam_pos.header.frame_id = "/camera_depth_frame";
            cam_pos.header.stamp = ros::Time(0);

            if (times == 0){
                cam_pos.point.x = 0.0;
                cam_pos.point.y = 0.0;
                cam_pos.point.z = 5.0;
            }
            else{
                cam_pos.point.x = X_pos/times;
                cam_pos.point.y = Y_pos/times;
                cam_pos.point.z = Z_pos/times;
                
            }

            try
            {
                tf::TransformListener pListener;
                pListener.waitForTransform("/camera_depth_frame", "/base_footprint", ros::Time(0), ros::Duration(3.0));
                pListener.transformPoint("/base_footprint", cam_pos, base_pos);
                ROS_INFO("cam_point: (%.2f, %.2f. %.2f) -----> base: (%.2f, %.2f, %.2f) at time %.2f",
                        cam_pos.point.x, cam_pos.point.y, cam_pos.point.z,
                        base_pos.point.x, base_pos.point.y, base_pos.point.z, base_pos.header.stamp.toSec());
            }
            catch(tf::TransformException& ex)
            {
                ROS_ERROR("Received an exception: %s", ex.what());
            }
            
            if (base_pos.point.z < 1.5){
                return ;
            }
            geometry_msgs::Pose pos;
            // XYZ对应关系是 Y Z X
            pos.position.y = base_pos.point.x;
            pos.position.z = base_pos.point.y;
            pos.position.x = base_pos.point.z - 2.0;

            pos.orientation.w = 1.0;
            position_pub.publish(pos);

        }
    }

    // 坐标回调函数
    void posviewCallback (const sensor_msgs::RegionOfInterest& msg){
        posinview.x_offset = msg.x_offset;
        posinview.y_offset = msg.y_offset;
        posinview.height   = msg.height;
        posinview.width    = msg.width;
        work = true;
    }
public:
    Depth(){
        // 参数获取
        posinview =  sensor_msgs::RegionOfInterest();

        position_pub = nh.advertise<geometry_msgs::Pose>("navigation_goal", 1000);
        pointcloud_sub = nh.subscribe("/camera/depth/points", 1, &Depth::pclCallback, this);
        posview_sub = nh.subscribe("roi", 1, &Depth::posviewCallback, this);
        
        camera_width = 640;
        camera_height = 480;
        cout<<"Start Work"<<endl;
    }
    ~Depth(){;}
};

int main(int argc, char **argv){
    ros::init (argc, argv, "depth_calculate");
    ROS_INFO("--------INITIALZING--------");
    Depth depth;
    ros::spin();
    return 0;
}
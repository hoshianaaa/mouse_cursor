#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/geometry.h>

#include "dobot_msgs/GetPose.h"

#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

static const std::string OPENCV_WINDOW = "Image window";
cv::Mat input_img_;
pcl::PointCloud<pcl::PointXYZ> input_cloud_, detect_cloud_, dobot_cloud_;
std::vector<double> detect_distances_, dobot_distances_;
std::string camera_frame_ = "camera_rgb_optical_frame";
std::string robot_frame_ = "base_link";
bool debug_distance_ = true;
tf::StampedTransform transform;

//マウス入力用のパラメータ
struct mouseParam {
    int x;
    int y;
    int event;
    int flags;
};

//コールバック関数
void CallBackFunc(int eventType, int x, int y, int flags, void* userdata)
{
    mouseParam *ptr = static_cast<mouseParam*> (userdata);

    ptr->x = x;
    ptr->y = y;
    ptr->event = eventType;
    ptr->flags = flags;
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  //std::cout << "image call back" << std::endl;
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  input_img_ = cv_ptr->image;
}

double distance(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  double dz = p1.z - p2.z;
  return std::sqrt(dx*dx + dy*dy + dz*dz);
}

void cloudCb(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  //std::cout << "cloud call back" << std::endl;
  sensor_msgs::PointCloud2 msg_transformed;
  pcl_ros::transformPointCloud("base_link", transform, *msg, msg_transformed);
  pcl::fromROSMsg( msg_transformed, input_cloud_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mouse_cursor");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub = it.subscribe("/camera/rgb/image_rect_color", 1, imageCb);
    ros::Subscriber cloud_sub = nh.subscribe("/camera/depth/points", 1, cloudCb);
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("detect_cloud", 1);
    ros::Publisher dobot_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("dobot_cloud", 1);
    
    ros::Rate loop_rate(10);

    // TFを受信するやつ
    tf::TransformListener listener;
    // base_linkとbase_laserの間のTFが取得できるまで繰り返す
    while(true){
      try{
        //取得できたらtransformに保存しておく
        listener.lookupTransform("base_link", camera_frame_, ros::Time(0), transform);
        ROS_INFO("I got a transform!");
        break;
      }
      catch(tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }
    }

    mouseParam mouseEvent;

    //表示するウィンドウ名
    cv::namedWindow(OPENCV_WINDOW,CV_WINDOW_AUTOSIZE);


    //コールバックの設定
    cv::setMouseCallback(OPENCV_WINDOW, CallBackFunc, &mouseEvent);

    while (ros::ok()) {

    if (input_img_.rows > 0 && input_img_.cols > 0)
    {
      cv::imshow(OPENCV_WINDOW, input_img_);
      cv::waitKey(100);

          if (mouseEvent.event == cv::EVENT_LBUTTONDOWN) {

              std::cout << "save detect pose" << std::endl;
              int x = mouseEvent.x;
              int y = mouseEvent.y;
              if (input_cloud_.size() > 0)
              {
                pcl::PointXYZ p = input_cloud_[input_cloud_.width * y + x];
                if(detect_cloud_.size()>1)
                {
                  detect_distances_.push_back(distance(p, detect_cloud_[detect_cloud_.size()]));
                }
                detect_cloud_.push_back(p);
                sensor_msgs::PointCloud2 detect_cloud_ros;
                pcl::toROSMsg(detect_cloud_, detect_cloud_ros);
                detect_cloud_ros.header.frame_id = robot_frame_;  
                cloud_pub.publish(detect_cloud_ros);
                std::cout << "publish" << std::endl;
              }
          }
          else if (mouseEvent.event == cv::EVENT_RBUTTONDOWN) {
            std::cout << "save dobot pose" << std::endl;
            ros::ServiceClient client;
            client = nh.serviceClient<dobot_msgs::GetPose>("/DobotServer/GetPose");
            dobot_msgs::GetPose srv;
            client.call(srv);

            pcl::PointXYZ p;
            p.x = srv.response.x / 1000; 
            p.y = srv.response.y / 1000; 
            p.z = srv.response.z / 1000; 
            if(dobot_cloud_.size()>1)
            {
              dobot_distances_.push_back(distance(p, dobot_cloud_[dobot_cloud_.size()]));
            }
            dobot_cloud_.push_back(p);
            sensor_msgs::PointCloud2 dobot_cloud_ros;
            pcl::toROSMsg(dobot_cloud_, dobot_cloud_ros);
            dobot_cloud_ros.header.frame_id = robot_frame_;

            dobot_cloud_pub.publish(dobot_cloud_ros);
          }
    }

    if (debug_distance_)
    {
      std::cout << "**detect**" << std::endl;
      for (int i=0;i<detect_distances_.size();i++)
      {
        std::cout << i+1 << " --- " << i+2 << ":" << detect_distances_[i] << std::endl;
      }
      std::cout << "**dobot**" << std::endl;
      for (int i=0;i<detect_distances_.size();i++)
      {
        std::cout << i+1 << " --- " << i+2 << ":" << detect_distances_[i] << std::endl;
      }
      std::cout << std::endl;
    }

    ros::spinOnce();
    //loop_rate.sleep();
    }
    return 0;
}

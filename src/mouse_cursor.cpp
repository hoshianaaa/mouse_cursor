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

static const std::string OPENCV_WINDOW = "Image window";
cv::Mat input_img_;
pcl::PointCloud<pcl::PointXYZ> input_cloud_, detect_cloud_, dobot_cloud_;
std::string camera_frame_ = "camera_rgb_optical_frame";

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

void cloudCb(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  //std::cout << "cloud call back" << std::endl;
  pcl::fromROSMsg( *msg, input_cloud_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mouse_cursor");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub = it.subscribe("/camera/rgb/image_rect_color", 1, imageCb);
    ros::Subscriber cloud_sub = nh.subscribe("/camera/depth/points", 1, cloudCb);
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("detect_cloud", 1);
    ros::Publisher dobot_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("detect_cloud", 1);
    
    ros::Rate loop_rate(10);


    mouseParam mouseEvent;

    //表示するウィンドウ名
    cv::namedWindow(OPENCV_WINDOW,CV_WINDOW_AUTOSIZE);


    //コールバックの設定
    cv::setMouseCallback(OPENCV_WINDOW, CallBackFunc, &mouseEvent);

    while (ros::ok()) {

	  //画像の表示
    if (input_img_.rows > 0 && input_img_.cols > 0)
    {
      cv::imshow(OPENCV_WINDOW, input_img_
      );
      cv::waitKey(100);

          //左クリックがあったら表示
          if (mouseEvent.event == cv::EVENT_LBUTTONDOWN) {
              //クリック後のマウスの座標を出力
              int x = mouseEvent.x;
              int y = mouseEvent.y;
              std::cout << x << " , " << y << std::endl;
              if (input_cloud_.size() > 0)
              {
                detect_cloud_.push_back(input_cloud_[input_cloud_.width * y + x]);
                sensor_msgs::PointCloud2 detect_cloud_ros;
                pcl::toROSMsg(detect_cloud_, detect_cloud_ros);
                detect_cloud_ros.header.frame_id = camera_frame_;  
                cloud_pub.publish(detect_cloud_ros);
                std::cout << "publish" << std::endl;
              }
          }
          //右クリックがあったら終了
          else if (mouseEvent.event == cv::EVENT_RBUTTONDOWN) {
            std::cout << "right button" << std::endl;
            ros::ServiceClient client;
            client = nh.serviceClient<dobot_msgs::GetPose>("/DobotServer/GetPose");
            dobot_msgs::GetPose srv;
            client.call(srv);

            std::cout << "x:" << srv.response.x;
            std::cout << " y:" << srv.response.y;
            std::cout << " z:" << srv.response.z << std::endl;

            pcl::PointXYZ p;
            p.x = srv.response.x / 100; 
            p.y = srv.response.y / 100; 
            p.z = srv.response.z / 100; 

            dobot_cloud_.push_back(p);
            
            sensor_msgs::PointCloud2 dobot_cloud_ros;
            pcl::toROSMsg(dobot_cloud_, dobot_cloud_ros);
            dobot_cloud_ros.header.frame_id = "base_link";

            dobot_cloud_pub.publish(dobot_cloud_ros);
          }
    }
    ros::spinOnce();
    //loop_rate.sleep();
    }
    return 0;
}

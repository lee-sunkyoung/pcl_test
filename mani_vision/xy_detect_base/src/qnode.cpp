/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/xy_detect_base/qnode.hpp"
#include <ros/network.h>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/String.h>
#include <string>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace xy_detect_base {
/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char **argv) : init_argc(argc), init_argv(argv) {}

QNode::~QNode() {
  if (ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init() {
  ros::init(init_argc, init_argv, "xy_detect_base");
  if (!ros::master::check()) {
    return false;
  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;

  // Add your ros communications here.
  image_transport::ImageTransport image(
      n); // 이미지 전송을 위한 ImageTransport 객체 생성
  subImage = image.subscribe("/camera/color/image_raw", 1,
                             &QNode::callbackImage, this); // 서브스크라이버

  image_transport::ImageTransport it_(n);
  std::string image_topic =
      n.resolveName("camera/aligned_depth_to_color/image_raw");
  sub_ = it_.subscribeCamera(image_topic, 1024, &QNode::callbackDepth, this);

  mani_vision_pub = n.advertise<mobile_base_msgs::mani_vision>("xy_detect", 1);

  start();
  return true;
}

void QNode::run() {
  ros::Rate loop_rate(33);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT
  rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::callbackImage(const sensor_msgs::ImageConstPtr &msg_img) {
  if (imgRaw == NULL && !isreceived) // imgRaw -> NULL, isreceived -> false
  {
    // ROS 이미지 메시지를 OpenCV Mat 형식으로 변환, 이미지 객체에 할당
    imgRaw = new cv::Mat(
        cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::RGB8)
            ->image);

    if (imgRaw != NULL) // imgRaw 변환 성공
    {
      Q_EMIT sigRcvImg(); // 이미지 수신을 알리는 시그널 발생
      isreceived = true;
    }
  }
}

void QNode::callbackDepth(const sensor_msgs::ImageConstPtr &image_msg,
                          const sensor_msgs::CameraInfoConstPtr &info_msg) {
  cv::Mat image;
  cv_bridge::CvImagePtr input_bridge;
  try {
    input_bridge = cv_bridge::toCvCopy(
        image_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    image = input_bridge->image;

    // Print Error message if image is null
    if (image.empty()) {
      ROS_INFO("Could not read image");
    }

    // Apply identity filter using kernel
    cv::Mat bilateral;

    medianBlur(image, bilateral, 3);

    cv::Mat kernel1 = (cv::Mat_<double>(3, 3) << 2, 2, 2, 0, 0, 0, -2, -2, -2);
    cv::Mat garo;
    filter2D(bilateral, garo, -1, kernel1, cv::Point(-1, -1), 0, 4);

    cv::Mat kernel2 = (cv::Mat_<double>(3, 3) << 2, 0, -2, 2, 0, -2, 2, 0, -2);
    cv::Mat sero;
    filter2D(bilateral, sero, -1, kernel2, cv::Point(-1, -1), 0, 4);

    cv::Mat mask = garo + sero;
    // filter2D(bilateral, mask, -1 , kernel3, cv::Point(-1, -1), 0, 4);
    cv::Mat kernel3 = (cv::Mat_<double>(3, 3) << 0, 0, 0, 0, 0, 0, 0, 0, 0);
    cv::Mat kernel4 = (cv::Mat_<double>(3, 3) << 0, 0, 0, 0, 1, 0, 0, 0, 0);

    // for (int a; mask.rows > a; a++) {
    //   for (int b; mask.cols > b; b++) {
    //     if (mask.at<short int>(cv::Point(b,a))<300) {
    //       mask.at<short int>(cv::Point(b,a))=0;
    //     } 
    //     else if(mask.at<short int>(cv::Point(b,a))>300){
    //       mask.at<short int>(cv::Point(b,a))*=2;
    //     }
    //   }
    // }

    cv::Mat mfilter;
    medianBlur(mask, mfilter, 3);
    identity = mfilter;

  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("[draw_frames] Failed to convert image");
    return;
  }

  // 이미지의 중심 좌표 계산
  int center_x = image.cols / 2;
  int center_y = image.rows / 2;

  max_depth = std::numeric_limits<short int>::min();
  min_depth = std::numeric_limits<short int>::max();

  for (int y = 0; y < image.rows; ++y) {
    for (int x = 0; x < image.cols; ++x) {
      float depth = image.at<short int>(cv::Point(x, y));

      if (depth == 0)
        continue;

      if (depth > max_depth) {
        max_depth = depth;
      }
      if (depth < min_depth) {
        min_depth = depth;
      }
    }
  }
  // std::cout << "in_cam_check" << std::endl;
}

} // namespace xy_detect_base

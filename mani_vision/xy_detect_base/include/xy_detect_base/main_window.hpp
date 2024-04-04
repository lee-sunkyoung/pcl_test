/**
 * @file /include/xy_detect_base/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date November 2010
 **/
#ifndef xy_detect_base_MAIN_WINDOW_H
#define xy_detect_base_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <vector>              // 벡터 관련 헤더 파일
#include "std_msgs/String.h"   // ROS 표준 메시지 타입을 사용하는 헤더 파일
#include <opencv2/opencv.hpp>  // OpenCV 라이브러리의 기본 헤더 파일
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace xy_detect_base
{
/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget* parent = 0);
  ~MainWindow();
  cv::Mat clone_mat;   // 원본 이미지 복사
  cv::Mat gray_clone;  // 흑백 이미지의 클론
  int value_hsv[6];    // hsv

  float diff_x = 0;
  float diff_y = 0;
  float img_center_x = 320.0;
  float img_center_y = 180.0;
  bool in_cam_check = false;

  bool in_center_x = false;
  bool in_center_y = false;
  mobile_base_msgs::mani_vision mvis;

public Q_SLOTS:
  void slotUpdateImg();

  void blue_img(cv::Mat& img);
  void Find_Binary_img(cv::Mat& img);
  cv::Mat Binary(cv::Mat& img, int val[]);

  void mvis_pub();

private:
  Ui::MainWindowDesign ui;
  QNode qnode;
};

}  // namespace xy_detect_base

#endif  // xy_detect_base_MAIN_WINDOW_H

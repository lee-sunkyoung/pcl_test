/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/xy_detect_base/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace xy_detect_base
{
using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget* parent) : QMainWindow(parent), qnode(argc, argv)
{
  ui.setupUi(this);  // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

  setWindowIcon(QIcon(":/images/icon.png"));

  qnode.init();

  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  QObject::connect(&qnode, SIGNAL(sigRcvImg()), this, SLOT(slotUpdateImg()));
}

MainWindow::~MainWindow()
{
}

/*****************************************************************************
** Functions
*****************************************************************************/
void MainWindow::slotUpdateImg()
{
  clone_mat = qnode.imgRaw->clone();                                            // 원본 이미지 복사
  cv::resize(clone_mat, clone_mat, cv::Size(640, 360), 0, 0, cv::INTER_CUBIC);  // 이미지 크기 조정
  // cv::cvtColor(clone_mat, gray_clone, cv::COLOR_BGR2GRAY);  // BGR 이미지를 그레이스케일로 변환

  Find_Binary_img(clone_mat);
  blue_img(clone_mat);

  mvis_pub();

  QImage RGB_im((const unsigned char*)(clone_mat.data), clone_mat.cols, clone_mat.rows, QImage::Format_RGB888);
  ui.label->setPixmap(QPixmap::fromImage(RGB_im));

  ui.m_1->display(mvis.difference[0]);
  ui.m_2->display(mvis.difference[1]);
  ui.m_3->display(mvis.difference[2]);
  ui.m_4->display(mvis.in_cam);
  ui.m_5->display(mvis.in_center);
  
  ui.m_6->display(mvis.center_depth);
  ui.m_7->display(mvis.high_depth);
  ui.m_8->display(mvis.low_depth);

  delete qnode.imgRaw;  // 동적 할당된 원본 이미지 메모리 해제
  qnode.imgRaw = NULL;
  qnode.isreceived = false;  // 이미지 수신 플래그 재설정
}

//파란색 이미지
void MainWindow::blue_img(cv::Mat& img)
{

//HHHHHHHHHHHHHHHHHHEEEEEEEEEEEEEEEERRRRRRRRRRRRRRRRRREEEEEEEEEEEEEEEEEEEE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
//HHHHHHHHHHHHHHHHHHEEEEEEEEEEEEEEEERRRRRRRRRRRRRRRRRREEEEEEEEEEEEEEEEEEEE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
//HHHHHHHHHHHHHHHHHHEEEEEEEEEEEEEEEERRRRRRRRRRRRRRRRRREEEEEEEEEEEEEEEEEEEE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//

  QImage binaryQImage(qnode.identity.data, qnode.identity.cols, qnode.identity.rows, qnode.identity.step, QImage::Format_Grayscale8);
   ui.test->setPixmap(QPixmap::fromImage(binaryQImage));

//HHHHHHHHHHHHHHHHHHEEEEEEEEEEEEEEEERRRRRRRRRRRRRRRRRREEEEEEEEEEEEEEEEEEEE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
//HHHHHHHHHHHHHHHHHHEEEEEEEEEEEEEEEERRRRRRRRRRRRRRRRRREEEEEEEEEEEEEEEEEEEE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
//HHHHHHHHHHHHHHHHHHEEEEEEEEEEEEEEEERRRRRRRRRRRRRRRRRREEEEEEEEEEEEEEEEEEEE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//



  in_cam_check = false;
  qnode.in_center_check = false;

  int blue_hsw_value[6] = { 0, 187, 60, 40, 255, 255 };  // low h, low s, low v, high h, high s, high v
  cv::Mat blue_img = Binary(img, blue_hsw_value);        // 파란색 이진화

  // QImage binaryQImage(blue_img.data, blue_img.cols, blue_img.rows, blue_img.step, QImage::Format_Grayscale8);
  // ui.test->setPixmap(QPixmap::fromImage(binaryQImage));

  cv::Mat labels, stats, centroids;
  int cnt =
      connectedComponentsWithStats(blue_img, labels, stats, centroids);  // 레이블 영역의 통계, 무게 중심 좌표 정보
  // cout << "number of labels : " << cnt << endl;

  // 객체가 존재하는 경우
  if (cnt > 1)
  {
    in_cam_check = true;
  }

  // 각 객체 영역에 바운딩 박스 표시하기
  for (int i = 1; i < cnt; i++)
  {
    int left = stats.at<int>(i, cv::CC_STAT_LEFT);
    int top = stats.at<int>(i, cv::CC_STAT_TOP);
    int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
    int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);

    rectangle(img, cv::Point(left, top), cv::Point(left + width, top + height), cv::Scalar(0, 255, 0),
              2);  // x,y,가로,세로 크기
    putText(img, std::to_string(i), cv::Point(left - 5, top + 5), 5, 1, cv::Scalar(255, 255, 255));

    // 레이블링된 물체의 중심 좌표
    float object_center_x = centroids.at<double>(i, 0);
    float object_center_y = centroids.at<double>(i, 1);

    // 이미지 중심과 레이블링된 물체 중심 간의 차이 계산
    diff_x = object_center_x - img_center_x;
    diff_y = object_center_y - img_center_y;

    if (diff_x < 15 && diff_x > -15)
    {
      in_center_x = true;
    }
    else
    {
      in_center_x = false;
    }

    if (diff_y < 15 && diff_y > -15)
    {
      in_center_y = true;
    }
    else
    {
      in_center_y = false;
    }

    qnode.in_center_check = (in_center_x && in_center_y);

    //   std::cout << in_cam_check << std::endl;
    //   std::cout << in_center_check << std::endl;
    //   std::cout << "Object " << i << " center offset: x = " << diff_x << ", y = " << diff_y << std::endl;
  }
}  // namespace xy_detect_base

// 이진화 찾는 함수
void MainWindow::Find_Binary_img(cv::Mat& img)
{
  // 표지판 이진화
  cv::Mat F_Image = Binary(img, value_hsv);

  // 이진 이미지를 표시
  // QImage binaryQImage(F_Image.data, F_Image.cols, F_Image.rows, F_Image.step, QImage::Format_Grayscale8);
  // ui.test->setPixmap(QPixmap::fromImage(binaryQImage));

  // 슬라이더
  value_hsv[0] = ui.horizontalSlider_7->value();   // low h
  value_hsv[1] = ui.horizontalSlider_8->value();   // low s
  value_hsv[2] = ui.horizontalSlider_9->value();   // low v
  value_hsv[3] = ui.horizontalSlider_10->value();  // high h
  value_hsv[4] = ui.horizontalSlider_11->value();  // high s
  value_hsv[5] = ui.horizontalSlider_12->value();  // high v

  // 슬라이더 값
  ui.s_1->display(value_hsv[0]);
  ui.s_2->display(value_hsv[1]);
  ui.s_3->display(value_hsv[2]);
  ui.s_4->display(value_hsv[3]);
  ui.s_5->display(value_hsv[4]);
  ui.s_6->display(value_hsv[5]);
}

// 이진화
cv::Mat MainWindow::Binary(cv::Mat& img, int val[])
{
  // 복사 이미지 HSV로 변환
  cv::Mat hsvImg;
  cv::cvtColor(img, hsvImg, cv::COLOR_BGR2HSV);

  // Gaussian blur로 반사율 쥴이기
  cv::Mat blurredImage;
  cv::GaussianBlur(hsvImg, blurredImage, cv::Size(7, 7), 3);

  
  cv::Mat mask = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5), cv::Point(2,2));
  cv::erode(blurredImage, blurredImage, mask, cv::Point(-1, -1), 3);

  // HSV 이미지를 사용하여 범위 내의 색상을 임계값으로 설정
  cv::Scalar lower(val[0], val[1], val[2]);
  cv::Scalar upper(val[3], val[4], val[5]);
  cv::Mat binaryImage;
  cv::inRange(blurredImage, lower, upper, binaryImage);

  return binaryImage;
}

void MainWindow::mvis_pub()
{
  mvis.difference[0] = diff_x;
  mvis.difference[1] = diff_y;
  mvis.difference[2] = 0;
  mvis.in_cam = in_cam_check;
  mvis.in_center = qnode.in_center_check;

  mvis.center_depth = qnode.depth_in_mm;
  // 나중에 수정할 부분
  mvis.high_depth = qnode.max_depth;
  mvis.low_depth = qnode.min_depth;

  qnode.mani_vision_pub.publish(mvis);
}

}  // namespace xy_detect_base

#ifndef DVSCONTROLLER_H
#define DVSCONTROLLER_H

#include "direct_visual_servoing.h"
#include <QThread>
#include <QSettings>
#include <QDir>
#include <QDebug>

class DVSController : public QThread
{
    Q_OBJECT
public:
    DVSController(const int resolution_x = 640, const int resolution_y = 480);
    ~DVSController();

    bool setDesiredImg(cv::Mat& image_gray_desired);
    bool setCurrentImg(cv::Mat& image_gray_current);

    cv::Mat getVelocity();
    bool isSuccess();


private:
    // --- 视觉伺服相关 ---
    Direct_Visual_Servoing *DVS_;
    void getParametersVS(double& lambda
                         , double& epsilon
                         , cv::Mat& imageGrayDesired
                         , cv::Mat& imageDepthDesired
                         , cv::Mat& cameraIntrinsic
                         , cv::Mat& poseDesired);
    cv::Mat rgbImageOperate(cv::Mat& image_rgb);
    cv::Mat depthImageOperate(cv::Mat& image_depth);
    bool get_image_gray_current(cv::Mat& img_new);
    bool get_image_depth_current(cv::Mat& img_new);
    Mat Adjoint(const Mat& T);
    Mat VecToso3(const Mat& vec);
    void TransToRp(const Mat& T, Mat& R, Mat& p);

private:
    void run() override;


private:
    cv::Mat image_gray_current_;
    cv::Mat image_gray_desired_;
    cv::Mat Ve_;
};

#endif // DVSCONTROLLER_H

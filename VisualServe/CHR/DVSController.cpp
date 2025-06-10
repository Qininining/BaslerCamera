#include "DVSController.h"
#include "chrtools.hpp"
// #include "modern_robotics_lib.h"

#define _STR(x) #x
#define STRINGIFY(x)  _STR(x)

DVSController::DVSController(const int resolution_x, const int resolution_y)
{
    this->DVS_ = new Direct_Visual_Servoing(resolution_x, resolution_y);

    // this->get_image_gray_current(image_gray_current_);
    Ve_ = (cv::Mat_<double>(6, 1) <<0, 0, 0, 0, 0, 0);
    // cv::imshow("RGB Image1", image_gray_current_);

    // this->start();
}

DVSController::~DVSController()
{
    this->quit();
    this->wait();
}

void DVSController::run()
{
    while(!this->isInterruptionRequested())
    {
        // 获取当前图像
        Mat depth_current, img_current;
        // this->get_image_gray_current(img_current);
        this->get_image_depth_current(depth_current);
        img_current = image_gray_current_;

        // 设置当前图像
        this->DVS_->set_image_gray_current(img_current);
        this->DVS_->set_image_depth_current(depth_current);

        // std::cout << "\t\t\tImg::" << img_current.size() << std::endl;

        if(this->DVS_->flag_first_)
        {
            double lambda, epsilon;
            Mat img_desired, depth_desired, camera_intrinsic, pose_desired;
            this->getParametersVS(lambda, epsilon, img_desired, depth_desired, camera_intrinsic, pose_desired);
            image_gray_desired_ = img_desired;
            this->DVS_->init_VS(lambda, epsilon, img_desired, depth_desired, img_current, camera_intrinsic, pose_desired);
            this->DVS_->flag_first_ = false;
        }

        // std::cout << "\t\t\tImg11::" << depth_current.size() << std::endl;

        Mat camera_velocity = this->DVS_->get_camera_velocity();
        int Velocity_x = static_cast<int>(round(camera_velocity.at<double>(0,0) * 1e6));
        int Velocity_y = static_cast<int>(round(camera_velocity.at<double>(1,0) * 1e6));
        // qDebug() << "\t\t\t速度camera_velocity：" << Velocity_x << Velocity_y;

        // Tc = T * To
        cv::Mat T = (cv::Mat_<double>(4, 4) <<
                         -1, 0, 0, 0,
                         0, -1, 0, 0,
                         0, 0, 1, 10000000,
                         0, 0, 0, 1);
        cv::Mat Ad_T = this->Adjoint(T);
        cv::Mat Ad_T_inv = Ad_T.inv();
        cv::Mat object_velocity = Ad_T_inv * (camera_velocity);

        this->Ve_ = object_velocity;
        // this->Ve_ = camera_velocity;

        // 打印结果
        // std::cout << "Camera Velocity (Vc):\n" << camera_velocity << std::endl;
        // std::cout << "Object Velocity (Ve):\n" << object_velocity << std::endl;
        // chrtools::print_time_with_microseconds();
    }
}

void DVSController::getParametersVS(double& lambda, double& epsilon, cv::Mat& imageGrayDesired, cv::Mat& imageDepthDesired, cv::Mat& cameraIntrinsic, cv::Mat& poseDesired)
{
    QString VSDir = STRINGIFY(VS_DIR);
    qDebug() << "VSDir:" << VSDir;
    QSettings settings(VSDir + "/config/config.ini", QSettings::IniFormat);

    // 读取基本参数
    lambda = settings.value("Parameters/lambda", 0.5).toDouble(); // 默认值 0.5
    epsilon = settings.value("Parameters/epsilon", 0.1).toDouble();
    qDebug() << "lambda:" << lambda << "epsilon:" << epsilon;

    // 读取资源路径和文件名
    QString resourceLocation = VSDir + "img";
    QString imageRgbName = settings.value("Resource/image_rgb_desired_name", "target_rgb.png").toString();
    QString imageDepthName = settings.value("Resource/image_depth_desired_name", "target_depth.png").toString();

    // 构建完整文件路径
    QDir resourceDir(resourceLocation);
    QString rgbPath = resourceDir.absoluteFilePath(imageRgbName);
    QString depthPath = resourceDir.absoluteFilePath(imageDepthName);
    qDebug() << "RGB Image Path:" << rgbPath << "Depth Image Path:" << depthPath;

    // 加载并处理 RGB 图像
    cv::Mat imageRgb = cv::imread(rgbPath.toStdString(), cv::IMREAD_COLOR);
    if (imageRgb.empty()) {
        qWarning() << "Failed to load RGB image:" << rgbPath;
        return;
    }
    imageGrayDesired = rgbImageOperate(imageRgb); // 假设 rgbImageOperate 是自定义处理函数
    // cv::imshow("RGB Image1", imageGrayDesired);

    // 加载并处理深度图像
    cv::Mat imageDepthRaw = cv::imread(depthPath.toStdString(), cv::IMREAD_UNCHANGED);
    if (imageDepthRaw.empty()) {
        qWarning() << "Failed to load depth image:" << depthPath;
        return;
    }
    imageDepthDesired = depthImageOperate(imageDepthRaw); // 假设 depthImageOperate 是自定义处理函数
    // cv::imshow("Depth Image", imageDepthDesired);

    // 读取相机内参矩阵 (假设存储为列表格式)
    QVariantList intrinsicList = settings.value("Camera/intrinsic").toList();
    if (intrinsicList.size() != 9) {
        qWarning() << "Invalid camera intrinsic parameters!";
        return;
    }
    cameraIntrinsic = cv::Mat(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) {
        cameraIntrinsic.at<double>(i/3, i%3) = intrinsicList[i].toDouble();
    }

    // 读取期望位姿矩阵 (4x4)
    QVariantList poseList = settings.value("Pose/desired").toList();
    if (poseList.size() != 16) {
        qWarning() << "Invalid pose matrix!";
        return;
    }
    poseDesired = cv::Mat(4, 4, CV_64F);
    for (int i = 0; i < 16; ++i) {
        poseDesired.at<double>(i/4, i%4) = poseList[i].toDouble();
    }
}

cv::Mat DVSController::rgbImageOperate(cv::Mat& image_rgb) {
    // 检查输入有效性
    if (image_rgb.empty()) {
        qWarning() << "输入 RGB 图像为空!";
        return cv::Mat();
    }
    cv::Mat image_gray;
    cv::cvtColor(image_rgb, image_gray, cv::COLOR_BGR2GRAY);
    image_gray.convertTo(image_gray, CV_64FC1);
    image_gray = cv::Scalar(1.0) - image_gray / 255.0;

    return image_gray;
}

cv::Mat DVSController::depthImageOperate(cv::Mat& image_depth)
{
    Mat image_depth_return;
    image_depth.convertTo(image_depth_return, CV_64FC1);
    image_depth_return = image_depth_return / 1000.0;
    return image_depth_return;
}

bool DVSController::get_image_gray_current(cv::Mat& img_current)
{
    cv::Mat image_rgb = cv::imread("D:/Project/CHR/Git/MicroAssembly/VisualServe/img/RGB_current.png", cv::IMREAD_UNCHANGED);
    if (image_rgb.empty()) {
        qWarning() << "输入图像为空!";
        return false;
    }
    img_current = this->rgbImageOperate(image_rgb);
    return true;
}

bool DVSController::get_image_depth_current(cv::Mat& img_current)
{
    cv::Mat image_depth = cv::imread("D:/Project/CHR/Git/MicroAssembly/VisualServe/img/depth_Black.png", cv::IMREAD_UNCHANGED);
    if (image_depth.empty()) {
        qWarning() << "输入图像为空!";
        return false;
    }
    img_current = this->depthImageOperate(image_depth);
    return true;
}


bool DVSController::setDesiredImg(cv::Mat& image_gray_desired)
{
    std::cout << "image_gray_desired::" << image_gray_desired.size();
    image_gray_desired_ = rgbImageOperate(image_gray_desired);
    this->DVS_->set_image_gray_desired(image_gray_desired_);
    return true;
}
bool DVSController::setCurrentImg(cv::Mat& image_gray_current)
{
    // std::cout << "setCurrentImg::" << image_gray_current.size() << std::endl;
    image_gray_current_ = rgbImageOperate(image_gray_current);
    return true;
}

cv::Mat DVSController::getVelocity()
{
    return Ve_;
}

bool DVSController::isSuccess()
{
    return this->DVS_->is_success();
}

// 计算 Adjoint 矩阵 [AdT]
Mat DVSController::Adjoint(const Mat& T) {
    Mat R, p;
    TransToRp(T, R, p);

    Mat zero33 = Mat::zeros(3, 3, CV_64F);
    Mat Rp = VecToso3(p) * R;

    // 构建 6x6 Adjoint 矩阵
    Mat AdT = Mat::zeros(6, 6, CV_64F);

    // Top-left: R
    R.copyTo(AdT(Rect(0, 0, 3, 3)));

    // Top-right: zeros(3)
    zero33.copyTo(AdT(Rect(3, 0, 3, 3)));

    // Bottom-left: VecToso3(p)*R
    Rp.copyTo(AdT(Rect(0, 3, 3, 3)));

    // Bottom-right: R
    R.copyTo(AdT(Rect(3, 3, 3, 3)));

    return AdT;
}


// 将三维向量转换为 so(3) 反对称矩阵
Mat DVSController::VecToso3(const Mat& vec) {
    CV_Assert(vec.rows == 3 && vec.cols == 1);

    double wx = vec.at<double>(0);
    double wy = vec.at<double>(1);
    double wz = vec.at<double>(2);

    Mat so3 = (Mat_<double>(3, 3) <<
                   0, -wz, wy,
               wz, 0, -wx,
               -wy, wx, 0);

    return so3;
}

// 从 SE(3) 变换矩阵中提取 R 和 p
void DVSController::TransToRp(const Mat& T, Mat& R, Mat& p) {
    CV_Assert(T.rows == 4 && T.cols == 4);

    // 提取旋转矩阵 R (3x3)
    R = T(Rect(0, 0, 3, 3)).clone();

    // 提取平移向量 p (3x1)
    p = T(Rect(3, 0, 1, 3)).clone();
}

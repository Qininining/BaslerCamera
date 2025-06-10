#include "IBVS.h"
#include <stdexcept> // 用于 std::runtime_error
#include <cmath> // for fabs, etc.
#include <QDebug>
#include <iostream>

IBVS::IBVS() : fx_(0), fy_(0), cx_(0), cy_(0), lambda_(0), numFeatures_(0), epsilon_(0.0) // 初始化 epsilon_
{
    // 默认构造函数
    T_effector_to_camera_ = cv::Mat::eye(4, 4, CV_64F); // 初始化为单位矩阵
}

void IBVS::init(double fx, double fy, double cx, double cy, double lambda, int numFeatures, double epsilon)
{
    fx_ = fx;
    fy_ = fy;
    cx_ = cx;
    cy_ = cy;
    lambda_ = lambda;
    numFeatures_ = numFeatures;
    epsilon_ = epsilon; // 设置收敛阈值

    if (numFeatures_ <= 0) {
        throw std::runtime_error("Number of features must be positive.");
    }
    if (epsilon_ < 0) {
        throw std::runtime_error("Epsilon (convergence tolerance) must be non-negative.");
    }

    // 初始化 cv::Mat 对象
    currentFeatures_ = cv::Mat::zeros(2 * numFeatures_, 1, CV_64F);
    desiredFeatures_ = cv::Mat::zeros(2 * numFeatures_, 1, CV_64F);
    depths_ = cv::Mat::ones(numFeatures_, 1, CV_64F)  * 820.0; // 初始化为 numFeatures_ x 1 的 CV_64F 矩阵，值为 1.0
    interactionMatrix_ = cv::Mat::zeros(2 * numFeatures_, 6, CV_64F);
    LDesired_ = cv::Mat::zeros(2 * numFeatures_, 6, CV_64F); 
    LCurrent_ = cv::Mat::zeros(2 * numFeatures_, 6, CV_64F);

    if (T_effector_to_camera_.empty()) { // 确保它被初始化
        T_effector_to_camera_ = cv::Mat::eye(4, 4, CV_64F);
    }
}

void IBVS::setEffectorToCameraTransform(const cv::Mat& T_ec)
{
    if (T_ec.rows != 4 || T_ec.cols != 4 || T_ec.type() != CV_64F) {
        throw std::runtime_error("T_effector_to_camera (T_ec) must be a 4x4 CV_64F matrix.");
    }
    T_ec.copyTo(T_effector_to_camera_);
}

void IBVS::setCurrentFeatures(const cv::Mat& s)
{
    if (s.rows != 2 * numFeatures_ || s.cols != 1 || s.type() != CV_64F) {
        qDebug() << "\t\t\t\t" << s.rows << 2 * numFeatures_ << s.cols << s.type();
        throw std::runtime_error("Current features cv::Mat dimensions or type are incorrect. Expected (2*numFeatures x 1) of CV_64F.");
    }
    s.copyTo(currentFeatures_);
    this->computeLCurrentInternal(); // 更新 LCurrent_
}

void IBVS::setDesiredFeatures(const cv::Mat& sStar)
{
    if (sStar.rows != 2 * numFeatures_ || sStar.cols != 1 || sStar.type() != CV_64F) {
        throw std::runtime_error("Desired features cv::Mat dimensions or type are incorrect. Expected (2*numFeatures x 1) of CV_64F.");
    }
    sStar.copyTo(desiredFeatures_);
    this->computeLDesiredInternal(); // 更新 LDesired_
}

void IBVS::setDepths(const cv::Mat& zValues)
{
    if (zValues.rows != numFeatures_ || zValues.cols != 1 || zValues.type() != CV_64F) {
        throw std::runtime_error("Depths cv::Mat dimensions or type are incorrect. Expected (numFeatures x 1) of CV_64F.");
    }
    zValues.copyTo(depths_);
    // 如果特征已经设置，深度变化时需要同时更新 LCurrent 和 LDesired
    if (!currentFeatures_.empty() && currentFeatures_.rows == 2 * numFeatures_) {
        this->computeLCurrentInternal();
    }
    if (!desiredFeatures_.empty() && desiredFeatures_.rows == 2 * numFeatures_) {
        this->computeLDesiredInternal();
    }
}

cv::Mat IBVS::getError() const
{
    if (currentFeatures_.empty() || desiredFeatures_.empty()) {
         throw std::runtime_error("Features not initialized before calling getError.");
    }
    if (currentFeatures_.rows != 2 * numFeatures_ || desiredFeatures_.rows != 2*numFeatures_) {
        throw std::runtime_error("Feature vector sizes are inconsistent in getError.");
    }
    return currentFeatures_ - desiredFeatures_;
}

double IBVS::getMeanSquaredError() const
{
    if (numFeatures_ == 0) {
        return 0.0; // 没有特征点，均方误差为0
    }
    if (currentFeatures_.empty() || desiredFeatures_.empty()) {
        // 如果特征未设置，无法计算，可以抛出异常或返回一个指示错误的值 (例如 NaN 或 -1)
        throw std::runtime_error("Features not set, cannot compute Mean Squared Error.");
        // 或者 return std::numeric_limits<double>::quiet_NaN();
    }

    cv::Mat errorVector = this->getError(); // 获取特征误差向量 e = s - s*

    // e.t() * e 计算误差平方和 (dot product)
    cv::Mat errorSquaredSumMat = errorVector.t() * errorVector; // 结果是一个 1x1 的 CV_64F 矩阵

    if (errorSquaredSumMat.empty() || errorSquaredSumMat.rows != 1 || errorSquaredSumMat.cols != 1) {
        // 理论上不应该发生，但作为防御性编程
        throw std::runtime_error("Error in calculating sum of squared errors.");
    }

    double sumOfSquaredErrors = errorSquaredSumMat.at<double>(0, 0);
    int numberOfErrorElements = errorVector.rows; // 对于列向量，行数即为元素数量

    if (numberOfErrorElements == 0) { // 避免除以零，尽管 numFeatures_ > 0 时不会发生
        return 0.0;
    }

    return sumOfSquaredErrors / static_cast<double>(numberOfErrorElements);
}

bool IBVS::isConverged() const
{
    if (numFeatures_ == 0) {
        return true; // 如果没有特征点，可以认为已经“收敛”
    }
    if (currentFeatures_.empty() || desiredFeatures_.empty()) {
        return false; // 认为未收敛
    }

    double mse = this->getMeanSquaredError();
    // qDebug() << "error:\t\t" << mse ;
    return mse < epsilon_;
}

void IBVS::computeSingleLInternal(const cv::Mat& features, cv::Mat& L_matrix)
{
    if (numFeatures_ == 0) {
        L_matrix = cv::Mat::zeros(0, 6, CV_64F);
        throw std::runtime_error("没有特征点，无法计算 L 矩阵");
        return;
    }
    if (features.empty() || features.rows != 2 * numFeatures_ || features.cols != 1) {
        L_matrix = cv::Mat::zeros(2 * numFeatures_, 6, CV_64F);
        throw std::runtime_error("特征点矩阵维度不正确，应该是 (2*numFeatures x 1) 的 CV_64F 矩阵");
        return;
    }
    if (depths_.empty() || depths_.rows != numFeatures_ || depths_.cols != 1 || depths_.type() != CV_64F) {
        L_matrix = cv::Mat::zeros(2 * numFeatures_, 6, CV_64F);
        // std::cout << "depths_" << depths_;
        throw std::runtime_error("深度矩阵维度不正确，应该是 (numFeatures x 1) 的 CV_64F 矩阵");
        return;
    }

    L_matrix = cv::Mat::zeros(2 * numFeatures_, 6, CV_64F); 

    for (int i = 0; i < numFeatures_; ++i)
    {
        double u = features.at<double>(2 * i, 0);
        double v = features.at<double>(2 * i + 1, 0);
        double z = depths_.at<double>(i, 0); 

        if (std::fabs(z) < 1e-6) {
            L_matrix.row(2 * i).setTo(cv::Scalar(0));
            L_matrix.row(2 * i + 1).setTo(cv::Scalar(0));
            continue;
        }

        // L_matrix.at<double>(2 * i, 0) = -fx_ / z;
        // L_matrix.at<double>(2 * i, 1) = 0;
        // L_matrix.at<double>(2 * i, 2) = (u - cx_) / z;
        // L_matrix.at<double>(2 * i, 3) = (u - cx_) * (v - cy_) / fy_;
        // L_matrix.at<double>(2 * i, 4) = -(fx_ * fx_ + (u - cx_) * (u - cx_)) / fx_;
        // L_matrix.at<double>(2 * i, 5) = (v - cy_);

        // L_matrix.at<double>(2 * i + 1, 0) = 0;
        // L_matrix.at<double>(2 * i + 1, 1) = -fy_ / z;
        // L_matrix.at<double>(2 * i + 1, 2) = (v - cy_) / z;
        // L_matrix.at<double>(2 * i + 1, 3) = (fy_ * fy_ + (v - cy_) * (v - cy_)) / fy_;
        // L_matrix.at<double>(2 * i + 1, 4) = -(u - cx_) * (v - cy_) / fx_;
        // L_matrix.at<double>(2 * i + 1, 5) = -(u - cx_);

        L_matrix.at<double>(2 * i, 0) = -fx_ / z;
        L_matrix.at<double>(2 * i, 1) = 0;
        L_matrix.at<double>(2 * i, 2) = 0;
        L_matrix.at<double>(2 * i, 3) = 0;
        L_matrix.at<double>(2 * i, 4) = 0;
        L_matrix.at<double>(2 * i, 5) = 0;

        L_matrix.at<double>(2 * i + 1, 0) = 0;
        L_matrix.at<double>(2 * i + 1, 1) = -fy_ / z;
        L_matrix.at<double>(2 * i + 1, 2) = 0;
        L_matrix.at<double>(2 * i + 1, 3) = 0;
        L_matrix.at<double>(2 * i + 1, 4) = 0;
        L_matrix.at<double>(2 * i + 1, 5) = 0;
    }
}

void IBVS::computeLDesiredInternal()
{
    computeSingleLInternal(desiredFeatures_, LDesired_);
    std::cout << "desiredFeatures_:" << desiredFeatures_ << std::endl;
}

void IBVS::computeLCurrentInternal()
{
    computeSingleLInternal(currentFeatures_, LCurrent_);
    std::cout << "currentFeatures_:" << currentFeatures_ << std::endl;
}

void IBVS::computeInteractionMatrix()
{
    if (numFeatures_ == 0) {
        interactionMatrix_ = cv::Mat::zeros(0, 6, CV_64F);
        return;
    }
    // 确保 LCurrent_ 和 LDesired_ 已经被计算或更新
    // 在 setFeatures 和 setDepths 中完成
    if (LCurrent_.rows != 2 * numFeatures_ || LCurrent_.cols != 6) {
        // 尝试重新计算，以防万一
        if (!currentFeatures_.empty() && currentFeatures_.rows == 2 * numFeatures_ && !depths_.empty() && depths_.rows == numFeatures_) {
            this->computeLCurrentInternal();
        } else {
            LCurrent_ = cv::Mat::zeros(2 * numFeatures_, 6, CV_64F); // 设为零矩阵
            throw std::runtime_error("没有有效的特征或深度，设置为零矩阵");
        }
        if (LCurrent_.rows != 2 * numFeatures_ || LCurrent_.cols != 6) { // 再次检查
            throw std::runtime_error("LCurrent_ has incorrect dimensions or could not be recomputed in computeInteractionMatrix.");
        }
    }
    if (LDesired_.rows != 2 * numFeatures_ || LDesired_.cols != 6) {
        if (!desiredFeatures_.empty() && desiredFeatures_.rows == 2 * numFeatures_ && !depths_.empty() && depths_.rows == numFeatures_) {
            this->computeLDesiredInternal();
        } else {
            LDesired_ = cv::Mat::zeros(2 * numFeatures_, 6, CV_64F); // 设为零矩阵
        }
         if (LDesired_.rows != 2 * numFeatures_ || LDesired_.cols != 6) { // 再次检查
            throw std::runtime_error("LDesired_ has incorrect dimensions or could not be recomputed in computeInteractionMatrix.");
        }
    }

    // interactionMatrix_ = (LCurrent_ + LDesired_) / 2.0;
    interactionMatrix_ = LCurrent_;
    // std::cout << "LCurrent_:" << LCurrent_ << "LDesired_:" << LDesired_ << "interactionMatrix_:" << interactionMatrix_ << std::endl;
}

cv::Mat IBVS::computeCameraVelocity()
{
    if (numFeatures_ == 0) {
        return cv::Mat::zeros(6, 1, CV_64F); 
    }

    this->computeInteractionMatrix(); // 这会使用预计算的 LCurrent_ 和 LDesired_

    cv::Mat error = this->getError();
    if (error.empty() || error.rows != 2 * numFeatures_) {
        throw std::runtime_error("Error vector is not correctly computed or sized.");
    }

    cv::Mat l_inv;
    cv::invert(interactionMatrix_, l_inv, cv::DECOMP_SVD);

    cv::Mat cameraVelocity = -lambda_ * l_inv * error;

    return cameraVelocity;
}

cv::Mat IBVS::convertEyeInHandToEyeOnHand(const cv::Mat& L_matrix) const {
    if (T_effector_to_camera_.empty() || T_effector_to_camera_.rows != 4 || T_effector_to_camera_.cols != 4) {
        throw std::runtime_error("T_effector_to_camera_ is not set or invalid. Call setEffectorToCameraTransform() first.");
    }
    // 检查输入 L_matrix 的维度 (可选，但推荐)
    if (L_matrix.empty() || L_matrix.cols != 6) { // 行数可以是 2*numFeatures_
         throw std::runtime_error("Input L_matrix for convertEyeInHandToEyeOnHand must have 6 columns.");
    }

    // cv::Mat Ad_T = this->Adjoint(this->T_effector_to_camera_);
    cv::Mat L_matrix_Object = -1 * L_matrix;
    // cv::Mat L_EyeOnHand = L_matrix_Object * Ad_T;

    // std::cout << "Ad_T:" << Ad_T << std::endl << "L_matrix_Object:" << L_matrix_Object << std::endl;

    return L_matrix_Object;
}

cv::Mat IBVS::getObjectVelocity()
{
    if (numFeatures_ == 0) {
        return cv::Mat::zeros(6, 1, CV_64F);
    }

    this->computeInteractionMatrix(); // 预计算 interactionMatrix_, LCurrent_ 和 LDesired_

    cv::Mat error = this->getError();
    if (error.empty() || error.rows != 2 * numFeatures_) {
        throw std::runtime_error("Error vector is not correctly computed or sized.");
    }

    cv::Mat L_EyeOnHand = this->convertEyeInHandToEyeOnHand(interactionMatrix_);
    // cv::Mat l_inv;
    // cv::invert(L_EyeOnHand, l_inv, cv::DECOMP_SVD);
    cv::Mat A = L_EyeOnHand;
    cv::Mat A_T;
    cv::transpose(A, A_T);  // 计算 A 的转置 Aᵀ (6x2)

    cv::Mat AAT = A * A_T;  // 计算 A Aᵀ (2x2)
    cv::Mat AAT_inv;
    cv::invert(AAT, AAT_inv, cv::DECOMP_LU);  // 计算 (A Aᵀ)⁻¹

    cv::Mat A_plus = A_T * AAT_inv;  // A⁺ = Aᵀ (A Aᵀ)⁻¹ (6x2)

    std::cout << "\nLeft Pseudo-Inverse (A⁺):\n" << A_plus << std::endl;



    cv::Mat l_inv = A_plus;



    cv::Mat I_check = L_EyeOnHand * l_inv;  // 应该接近 2x2 单位矩阵
    std::cout << "L_EyeOnHand * l_inv:\n" << I_check << std::endl;

    std::cout << "L_EyeOnHand:\n" << L_EyeOnHand << std::endl << "l_inv:\n" << l_inv << std::endl << "error\n" << error << std::endl;

    cv::Mat cameraVelocity = -lambda_ * l_inv * error;

    cv::Mat Ad_T = this->Adjoint(this->T_effector_to_camera_);
    cv::Mat Ad_T_inv = Ad_T.inv();
    cv::Mat object_velocity = Ad_T_inv * (cameraVelocity);

    std::cout << "cameraVelocity:\n" << cameraVelocity << "Ad_T:\n" << Ad_T << std::endl;

    return object_velocity;
}



// 计算 Adjoint 矩阵 [AdT]
cv::Mat IBVS::Adjoint(const cv::Mat& T) const { // 标记为 const
    cv::Mat R, p;
    this->TransToRp(T, R, p); // TransToRp 也需要是 const

    // cv::Mat zero33 = cv::Mat::zeros(3, 3, CV_64F); // 不再需要，可以直接在构建时使用 R 和 Rp
    cv::Mat Rp_times_R = this->VecToso3(p) * R; // VecToso3 也需要是 const

    // 构建 6x6 Adjoint 矩阵
    cv::Mat AdT = cv::Mat::zeros(6, 6, CV_64F);

    // Top-left: R
    R.copyTo(AdT(cv::Rect(0, 0, 3, 3)));

    // Top-right: zeros(3,3) - 默认已经是0，但明确设置一下也可以
    // cv::Mat::zeros(3, 3, CV_64F).copyTo(AdT(cv::Rect(3, 0, 3, 3))); // 如果需要明确

    // Bottom-left: [p]*R
    Rp_times_R.copyTo(AdT(cv::Rect(0, 3, 3, 3)));

    // Bottom-right: R
    R.copyTo(AdT(cv::Rect(3, 3, 3, 3)));

    return AdT;
}


// 将三维向量转换为 so(3) 反对称矩阵
cv::Mat IBVS::VecToso3(const cv::Mat& vec) const { // 标记为 const
    CV_Assert(vec.rows == 3 && vec.cols == 1 && vec.type() == CV_64F);

    double wx = vec.at<double>(0,0);
    double wy = vec.at<double>(1,0);
    double wz = vec.at<double>(2,0);

    cv::Mat so3 = (cv::Mat_<double>(3, 3) <<
                       0, -wz, wy,
                   wz, 0, -wx,
                   -wy, wx, 0);

    return so3;
}

// 从 SE(3) 变换矩阵中提取 R 和 p
void IBVS::TransToRp(const cv::Mat& T, cv::Mat& R, cv::Mat& p) const { // 标记为 const
    CV_Assert(T.rows == 4 && T.cols == 4 && T.type() == CV_64F);

    R = T(cv::Rect(0, 0, 3, 3)).clone();
    p = T(cv::Rect(3, 0, 1, 3)).clone();
}


#ifndef IBVS_H
#define IBVS_H

#include <opencv2/opencv.hpp> // 使用 OpenCV
#include <vector>
#include <string>

class IBVS
{
public:
    // 相机内参
    double fx_;
    double fy_;
    double cx_;
    double cy_;
    // 控制增益
    double lambda_;

    // 特征点相关
    cv::Mat currentFeatures_; // 当前特征 s = [u1, v1, u2, v2, ...]^T (列向量)
    cv::Mat desiredFeatures_; // 期望特征 s_star = [u1*, v1*, u2*, v2*, ...]^T (列向量)
    cv::Mat depths_;      // 每个特征点的深度 Z_i (numFeatures_ x 1, CV_64F)

    // 交互矩阵
    cv::Mat interactionMatrix_; // L_s

    // 构造函数
    IBVS();

    /**
     * @brief 初始化IBVS参数
     * @param fx 相机x轴焦距
     * @param fy 相机y轴焦距
     * @param cx 光心x坐标
     * @param cy 光心y坐标
     * @param lambda 控制增益
     * @param numFeatures 特征点数量
     * @param epsilon 收敛判断的均方误差阈值
     */
    void init(double fx, double fy, double cx, double cy, double lambda, int numFeatures, double epsilon);

    /**
     * @brief 设置当前观测到的特征点
     * @param s cv::Mat 类型, 维度为 (2*numFeatures)x1, 格式为 [u1, v1, u2, v2, ...]
     */
    void setCurrentFeatures(const cv::Mat& s);

    /**
     * @brief 设置期望的特征点
     * @param sStar cv::Mat 类型, 维度为 (2*numFeatures)x1, 格式为 [u1*, v1*, u2*, v2*, ...]
     */
    void setDesiredFeatures(const cv::Mat& sStar);

    /**
     * @brief 设置每个特征点的深度
     * @param zValues cv::Mat 类型, 维度为 numFeatures_x1, CV_64F, 包含每个特征点的深度值
     */
    void setDepths(const cv::Mat& zValues);

    /**
     * @brief 设置相机相对于末端执行器的变换矩阵 (eye-in-hand)
     * @param T_ec 4x4 的变换矩阵 (CV_64F)
     */
    void setEffectorToCameraTransform(const cv::Mat& T_ec);

    /**
     * @brief 计算交互矩阵 L_s
     *        L_s 的维度为 (2*numFeatures) x 6
     */
    void computeInteractionMatrix();

    /**
     * @brief 计算相机速度指令
     * @return cv::Mat 类型, 维度为 6x1, 表示相机速度 [vx, vy, vz, wx, wy, wz]^T
     */
    cv::Mat computeCameraVelocity();

    /**
     * @brief 获取当前特征与目标特征之间的误差
     * @return cv::Mat 类型, 维度为 (2*numFeatures)x1
     */
    cv::Mat getError() const;

    /**
     * @brief 计算当前特征误差的均方误差 (MSE)
     * @return double 类型的均方误差值
     */
    double getMeanSquaredError() const;

    /**
     * @brief 判断视觉伺服是否收敛
     *        通过比较 getMeanSquaredError() 的结果与初始化时设定的 epsilon_ 阈值来判断。
     * @return 如果均方误差小于 epsilon_ 则返回 true，否则返回 false
     */
    bool isConverged() const;

    /**
     * @brief 使用存储的 T_effector_to_camera_ 转换交互矩阵 L_matrix
     *        将相机坐标系下的交互矩阵转换为末端执行器坐标系下的交互矩阵。
     * @param L_matrix 相机坐标系下的交互矩阵 ( (2*numFeatures) x 6 )
     * @return cv::Mat 末端执行器坐标系下的交互矩阵
     */
    cv::Mat convertEyeInHandToEyeOnHand(const cv::Mat& L_matrix) const;

    /**
     * @brief 计算给定相机速度时，图像特征点在图像平面上的视在速度 (s_dot)。
     * @return cv::Mat 图像特征点的速度向量 ( (2*numFeatures)x1 CV_64F ), [u1_dot, v1_dot, ..., un_dot, vn_dot]^T.
     */
    cv::Mat getObjectVelocity();


private:
    int numFeatures_; // 特征点数量
    double epsilon_;  // 收敛判断的均方误差阈值
    cv::Mat LDesired_; // 预先计算的期望特征的交互矩阵部分
    cv::Mat LCurrent_; // 预先计算的当前特征的交互矩阵部分
    cv::Mat T_effector_to_camera_; // 相机相对于末端执行器的变换

    /**
     * @brief 内部函数，用于计算并更新 LDesired_
     */
    void computeLDesiredInternal();

    /**
     * @brief 内部函数，用于计算并更新 LCurrent_
     */
    void computeLCurrentInternal();

    /**
     * @brief 内部辅助函数，用于计算单个交互矩阵（LCurrent 或 LDesired）
     * @param features 要用于计算的特征点 (currentFeatures_ 或 desiredFeatures_)
     * @param L_matrix 要更新的交互矩阵 (LCurrent_ 或 LDesired_)
     */
    void computeSingleLInternal(const cv::Mat& features, cv::Mat& L_matrix);

    /**
     * @brief 内部辅助函数，用于计算 Adjoint 矩阵
     * @param T SE(3) 变换矩阵
     * @return cv::Mat 类型的 Adjoint 矩阵
     */
    cv::Mat Adjoint(const cv::Mat& T) const;

    /**
     * @brief 内部辅助函数，用于将向量转换为 so(3) 矩阵
     * @param vec 3x1 向量
     * @return cv::Mat 类型的 3x3 so(3) 矩阵
     */
    cv::Mat VecToso3(const cv::Mat& vec) const;

    /**
     * @brief 内部辅助函数，用于从变换矩阵中提取旋转矩阵和平移向量
     * @param T SE(3) 变换矩阵
     * @param R 输出的旋转矩阵
     * @param p 输出的平移向量
     */
    void TransToRp(const cv::Mat& T, cv::Mat& R, cv::Mat& p) const;
};

#endif // IBVS_H

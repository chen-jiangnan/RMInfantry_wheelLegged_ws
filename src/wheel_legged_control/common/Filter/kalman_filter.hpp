#pragma once

#include <Eigen/Dense>
#include <vector>

class KalmanFilter
{
public:
    KalmanFilter(int x_dim, int u_dim, int z_dim);

    // 核心接口（等价 Kalman_Filter_Update）
    Eigen::VectorXf update();

    // 预测 / 更新分离（可选用）
    void predict();
    void correct();

    // ================= 参数与状态 =================
    Eigen::VectorXf x;     // x(k|k)
    Eigen::VectorXf u;     // 控制输入

    Eigen::MatrixXf P;     // 协方差
    Eigen::MatrixXf F;     // 状态转移
    Eigen::MatrixXf Q;     // 过程噪声
    Eigen::MatrixXf B;     // 控制矩阵

    // =============== 量测相关 =====================
    Eigen::VectorXf measured_vector;   // 原始量测（zSize）

    std::vector<int>   measurement_map;     // MeasurementMap
    std::vector<float> measurement_degree;  // MeasurementDegree
    std::vector<float> R_diag;               // R 对角元素
    std::vector<float> state_min_variance;   // 最小方差限制

    bool use_auto_adjustment = true;

    const Eigen::VectorXf& getZVector() const {return z;}
    const Eigen::MatrixXf& getHMatrix() const {return H;}
    const Eigen::MatrixXf& getRMatrix() const {return R;}
    const Eigen::MatrixXf& getKMatrix() const {return K;}
    void debugPrint(const std::string& title = "") const;

private:
    void adjustMeasurement();

    int x_dim_;
    int u_dim_;
    int z_dim_;
    int valid_z_num_;

    // 动态矩阵
    Eigen::VectorXf z;
    Eigen::MatrixXf H;
    Eigen::MatrixXf R;
    Eigen::MatrixXf K;
};
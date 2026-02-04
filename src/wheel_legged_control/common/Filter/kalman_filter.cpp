#include "kalman_filter.hpp"
#include <iostream>
#include <iomanip>

KalmanFilter::KalmanFilter(int x_dim, int u_dim, int z_dim)
    : x_dim_(x_dim), u_dim_(u_dim), z_dim_(z_dim), valid_z_num_(0)
{
    x = Eigen::VectorXf::Zero(x_dim_);
    P = Eigen::MatrixXf::Zero(x_dim_, x_dim_);
    F = Eigen::MatrixXf::Identity(x_dim_, x_dim_);
    Q = Eigen::MatrixXf::Zero(x_dim_, x_dim_);

    if (u_dim_ > 0)
    {
        u = Eigen::VectorXf::Zero(u_dim_);
        B = Eigen::MatrixXf::Zero(x_dim_, u_dim_);
    }

    measured_vector = Eigen::VectorXf::Zero(z_dim_);

    measurement_map.resize(z_dim_, 0);
    measurement_degree.resize(z_dim_, 0.0f);
    R_diag.resize(z_dim_, 0.0f);
    state_min_variance.resize(x_dim_, 0.0f);
}

void KalmanFilter::adjustMeasurement()
{
    valid_z_num_ = 0;
    for (int i = 0; i < z_dim_; ++i)
        if (measured_vector(i) != 0.0f)
            valid_z_num_++;

    if (valid_z_num_ == 0)
        return;

    z = Eigen::VectorXf::Zero(valid_z_num_);
    H = Eigen::MatrixXf::Zero(valid_z_num_, x_dim_);
    R = Eigen::MatrixXf::Zero(valid_z_num_, valid_z_num_);

    int row = 0;
    for (int i = 0; i < z_dim_; ++i)
    {
        if (measured_vector(i) != 0.0f)
        {
            z(row) = measured_vector(i);

            int state_idx = measurement_map[i] - 1;
            if (state_idx >= 0 && state_idx < x_dim_)
                H(row, state_idx) = measurement_degree[i];

            R(row, row) = R_diag[i];
            row++;
        }
    }

    measured_vector.setZero();
}

void KalmanFilter::predict()
{
    if (u_dim_ > 0)
        x = F * x + B * u;
    else
        x = F * x;

    P = F * P * F.transpose() + Q;
}

void KalmanFilter::correct()
{
    if (valid_z_num_ == 0)
        return;

    Eigen::MatrixXf S = H * P * H.transpose() + R;
    K = P * H.transpose() * S.inverse();

    x = x + K * (z - H * x);
    P = P - K * H * P;

    // 防止过度收敛
    for (int i = 0; i < x_dim_; ++i)
    {
        if (P(i, i) < state_min_variance[i])
            P(i, i) = state_min_variance[i];
    }
}

Eigen::VectorXf KalmanFilter::update()
{
    if (use_auto_adjustment)
        adjustMeasurement();
    else
        z = measured_vector;

    predict();
    correct();
    //std::cout<< x << std::endl;
    return x;
    
}

void KalmanFilter::debugPrint(const std::string& title) const
{
    auto printVector = [](const std::string& name, const Eigen::VectorXf& v)
    {
        std::cout << name << " = [ ";
        for (int i = 0; i < v.size(); ++i)
            std::cout << std::setw(10) << v(i) << " ";
        std::cout << "]\n";
    };

    auto printMatrix = [](const std::string& name, const Eigen::MatrixXf& m)
    {
        std::cout << name << " =\n";
        for (int i = 0; i < m.rows(); ++i)
        {
            std::cout << "  ";
            for (int j = 0; j < m.cols(); ++j)
                std::cout << std::setw(10) << m(i, j) << " ";
            std::cout << "\n";
        }
    };

    std::cout << "\n================ Kalman Filter Debug =================\n";
    if (!title.empty())
        std::cout << title << "\n";

    std::cout << "Dimensions: x=" << x_dim_
              << ", u=" << u_dim_
              << ", z=" << z_dim_
              << ", valid_z=" << valid_z_num_
              << "\n\n";

    // ================= 状态 =================
    printVector("x (state)", x);
    printVector("u (control)", u);

    // ================= 模型参数 =================
    printMatrix("F (state transition)", F);
    printMatrix("P (covariance)", P);
    printMatrix("Q (process noise)", Q);

    if (u_dim_ > 0)
        printMatrix("B (control matrix)", B);

    // ================= 量测相关 =================
    printVector("measured_vector (raw)", measured_vector);
    printVector("z (used)", z);

    printMatrix("H (measurement)", H);
    printMatrix("R (measurement noise)", R);
    printMatrix("K (kalman gain)", K);

    // ================= 量测映射信息 =================
    std::cout << "\nMeasurement Map:\n";
    for (size_t i = 0; i < measurement_map.size(); ++i)
    {
        std::cout << "  z[" << i << "] -> x[" << measurement_map[i] - 1
                  << "], degree = " << measurement_degree[i]
                  << ", R = " << R_diag[i] << "\n";
    }

    // ================= 状态方差限制 =================
    if (!state_min_variance.empty())
    {
        std::cout << "\nState min variance:\n";
        for (size_t i = 0; i < state_min_variance.size(); ++i)
        {
            std::cout << "  x[" << i << "] >= " << state_min_variance[i] << "\n";
        }
    }

    std::cout << "======================================================\n\n";
}

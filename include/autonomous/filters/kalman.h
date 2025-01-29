// #pragma once

// #include "Eigen/Dense"

// namespace filters {
// template <int N>
// class LinearKalmanFilter {
//     private:
//         Eigen::Matrix<float, N, N> A;
//         Eigen::Matrix<float, N, N> B;
//         Eigen::Vector<float, N> x;

//         Eigen::Matrix<float, N, N> error_cov;
//         Eigen::Matrix<float, N, N> process_cov;
//         Eigen::Matrix<float, N, N> measurement_cov;

//     public:
//         LinearKalmanFilter(const Eigen::Vector<float, N>& x, 
//             const Eigen::Matrix<float, N, N>& error_cov,
//             const Eigen::Matrix<float, N, N>& process_cov,
//             const Eigen::Matrix<float, N, N>& measurement_cov)
//             : x(x), error_cov(error_cov), process_cov(process_cov), measurement_cov(measurement_cov) {}

//         void predict(const Eigen::Vector<float, N>& u) {
//             x = A * x + B * u;
//             error_cov = A * error_cov * A.transpose() + process_cov;
//         }

//         void update(const Eigen::Vector<float, N>& z) {
//             Eigen::Matrix<float, N, N> K = error_cov * C.transpose() * (C * error_cov * C.transpose() + measurement_cov).inverse();
//             x = x + K * (z - C * x);
//             error_cov = (Eigen::Matrix<float, N, N>::Identity() - K * C) * error_cov;
//         }

//         Eigen::Vector<float, N> get_state(void) const { return x; }
//         Eigen::Matrix<float, N, N> get_error_cov(void) const { return error_cov; }
// };
// };
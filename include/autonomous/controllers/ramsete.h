// #pragma once

// #include <utility>

// namespace controllers {
// class Ramsete {
//     public:
//         float beta, zeta;
//         float desired_x, desired_y, desired_theta;
//         float desired_v, desired_w;
    
//         Ramsete(float beta, float zeta):
//             beta(beta), zeta(zeta) 
//         {
//             desired_x = 0;
//             desired_y = 0;
//             desired_theta = 0;
//         }

//         inline std::pair<float, float> get(bool use_vw = false);

//     static std::pair<float, float> quick_ramsete(float beta, float zeta, float x, float y, float theta, float v, float w);
//     static std::pair<float, float> quick_ramsete(float beta, float zeta, float x, float y, float theta);
// };
// } // namespace controllers
#include "autonomous/movement/simple/mpc.h"

#include "ansicodes.h"
#include "essential.h"
#include "nlopt/nlopt.hpp"
#include "autodiff/reverse/var.hpp"
#include "pros/rtos.hpp"
#include <vector>

using var = autodiff::Variable<double>;
using namespace movement::simple;

struct AggregateData {
    const std::vector<DiffdriveState<var>>& desired;
    const DiffdriveState<var>& state;
    const DiffdriveMPCParams& params;
    const DiffdrivePenalty& penalty;
};

template <typename Func, typename Vector, std::size_t... Indices>
auto get_derivatives_with_indices(Func&& f, const Vector& x_ad, std::index_sequence<Indices...>) {
    return autodiff::derivatives(f, autodiff::wrt(x_ad[Indices]...));
}

template <int N, typename Func, typename Vector>
auto get_derivatives(Func&& f, const Vector& x_ad) {
    return get_derivatives_with_indices(
        std::forward<Func>(f), x_ad, std::make_index_sequence<N>{}
    );
}

static float get_scaling(const MPCScaling& scaling, int i, int N) {
    switch (scaling) {
        case MPCScaling::LINEAR:
            return (float) (i + 1) / N;
        case MPCScaling::QUADRATIC:
            return pow(i + 1, 2) / pow(N, 2);
        case MPCScaling::EXPONENTIAL:
            return exp(i) / exp(N - 1);
        case MPCScaling::DECAYING:
            return exp(N - i) / exp(N - 1);
        default:
            return 1.0;
    }
}

template <typename T>
static T ad_diffdrive_costfunc(
    const int N, 
    const std::vector<T>& x, 
    const std::vector<DiffdriveState<T>>& desired,
    const DiffdriveState<T>& state,
    const DiffdriveMPCParams& params, 
    const DiffdrivePenalty& penalty) 
{
    auto copy = state;
    T cost = 0.0;
    for (int i = 0; i < N; i++) {
        T vl_change = (params.gain * x[2*i] - copy.vl) / params.tc;
        T vr_change = (params.gain * x[2*i+1] - copy.vr) / params.tc;
        copy.vl += vl_change * params.optimization_dt;
        copy.vr += vr_change * params.optimization_dt;
        T v = (2*M_PI/60.0) * params.linear_mult * (copy.vl + copy.vr) / 2.0;
        T w = (2*M_PI/60.0) * params.linear_mult * (copy.vl - copy.vr) / params.track_width;
        T half = w * params.optimization_dt / 2.0, travel = v * params.optimization_dt;
        T s = sinc(half);
        copy.x += travel * s * sin(copy.theta + half);
        copy.y += travel * s * cos(copy.theta + half);
        copy.theta += half * 2.0;

        const auto& dstate = desired[i];
        auto error = copy - dstate;
        cost += (error * error * penalty).sum() * get_scaling(params.scaling, i, N);
        cost += (vl_change * vl_change + vr_change * vr_change) * penalty.u_diff_penalty;
        cost += (x[2*i] * x[2*i] + x[2*i+1] * x[2*i+1]) * penalty.u_penalty / params.optimization_dt;
    }

    return cost;
}

template <int N>
static double nlopt_diffdrive_costfunc(unsigned int n, const double* x, double* grad, void* data) {
    auto& [desired, state, params, penality] = *static_cast<AggregateData*>(data);

    std::vector<var> x_ad; 
    for (int i = 0; i < n; i++) x_ad.push_back(x[i]);

	var f = ad_diffdrive_costfunc<var>(N, x_ad, desired, state, params, penality);

	if (grad) {
        auto grad_ad = get_derivatives<N>(f, x_ad);
        for (int i = 0; i < n; i++) grad[i] = autodiff::val(grad_ad[i]);
    }

	return autodiff::val(f);
}

template <int N>
void movement::simple::follow_recording(const std::string& path, DiffdriveMPCParams mpc_params, DiffdrivePenalty penalty) {
    FILE* file = fopen(path.c_str(), "r");
    if (!file) {
        std::cerr << "Failed to open file " << path << std::endl;
        return;
    }

    std::vector<float> times;
    std::vector<DiffdriveState<var>> desired;
    int n_poses = 0;

    char header[256];
    fscanf(file, "%s", header);

    float _ftime = -1;
    while (!feof(file)) {
        float time, x, y, theta, vl, vr;
        fscanf(file, "%f,%f,%f,%f,%f,%f", &time, &x, &y, &theta, &vl, &vr);
        printf("%f, %f, %f, %f, %f\n", x, y, theta, vl, vr);
        desired.push_back({(double)x, (double)y, (double)theta, (double)vl, (double)vr});
        if (_ftime == -1) _ftime = time;
        times.push_back(time - _ftime);
        n_poses++;
    }

    bool set_optim_time = mpc_params.optimization_dt == -1;
    int distance_parametrized_i = 0;
    int time_parametrized_i = 0;
    float time_integral = 0;
    float last_time = pros::micros() / 1e6f;
    std::vector<double> u(2 * N, 0.0);

    pros::delay(mpc_params.dt_guess * 1000);

    while (distance_parametrized_i != n_poses && time_parametrized_i != n_poses) {
        float dt = pros::micros() / 1e6f - last_time;
        last_time = pros::micros() / 1e6f;
        if (set_optim_time) mpc_params.optimization_dt = dt;

        if (dt > 0.05) {
            printf("%sWarning: dt is %f seconds, this is high\n", CPREFIX, dt);
        }

        while (distance_parametrized_i < n_poses - 1) {
            const auto& p = desired[distance_parametrized_i];

            float average_speed = autodiff::val((p.vl + p.vr) / 2.0);
            Eigen::Vector2f deriv = {
                sinf(autodiff::val(p.theta)) * average_speed, 
                cosf(autodiff::val(p.theta)) * average_speed
            };
            Eigen::Vector2f pos = {
                autodiff::val(p.x), 
                autodiff::val(p.y)
            };
            
            if (deriv.dot(pos - robot::pos()) > 0) break;
            distance_parametrized_i++;
        }

        int disagreement_factor = 5;
        bool use_time_scaling = false;
        float scale = 1.0f;

        if (use_time_scaling) {
            if (abs(time_parametrized_i - distance_parametrized_i) > disagreement_factor) { // disagreement
                // warp time
                float beta = (abs(time_parametrized_i - distance_parametrized_i) - disagreement_factor) / (float) disagreement_factor;
                scale = (sqrtf(fmaxf(0, 1 - beta * beta)) + expf(-2 * beta)) / 2.0f;
            }
        }

        time_integral += dt * scale;

        time_parametrized_i = std::max(
            std::upper_bound(times.begin(), times.end(), time_integral) - times.begin() - 1, 
            0
        );

        if (time_parametrized_i == n_poses - 1) break;

        DiffdriveState<var> state {
            robot::x(), robot::y(), robot::theta(), 
            robot::left_motors.get_filtered_velocity(), 
            robot::right_motors.get_filtered_velocity()
        };

        std::vector<DiffdriveState<var>> desired_local;
        for (int i = time_parametrized_i; i < time_parametrized_i + N; i++) {
            if (i >= desired.size()) break;
            desired_local.push_back(desired[i]);
        }
        while (desired_local.size() < N) {
            desired_local.push_back(desired_local.back());
        }
        AggregateData data {desired_local, state, mpc_params, penalty};

        nlopt::opt opt(mpc_params.alg, 2 * N);
        opt.set_ftol_rel(0.01);
        std::vector<double> lb(2 * N, -12); opt.set_lower_bounds(lb);
        std::vector<double> ub(2 * N, 12); opt.set_upper_bounds(ub);
        opt.set_min_objective(nlopt_diffdrive_costfunc<N>, &data);

        double minf;
        nlopt::result result = opt.optimize(u, minf);

        if (result < 0) {
            std::cerr << "NLOPT failed with code " << result << std::endl;
            break;
        }

        bool adjust_for_friction = false;

        float VL = (u[0] * 0.5 + u[2] * 0.25 + u[4] * 0.125) / 12.0f;
        float VR = (u[1] * 0.5 + u[3] * 0.25 + u[5] * 0.125) / 12.0f;
        printf("Time parametrized i: %d, Distance parametrized i: %d\n", time_parametrized_i, distance_parametrized_i);
        robot::chassis.set_voltage(
            VL + (adjust_for_friction ? mpc_params.kf * (VL > 0 ? 1 : -1) : 0),
            VR + (adjust_for_friction ? mpc_params.kf * (VR > 0 ? 1 : -1) : 0)
        );

        // robot::chassis.set_velocity(
        //     autodiff::val(desired[time_parametrized_i].vl) / 600,
        //     autodiff::val(desired[time_parametrized_i].vr) / 600
        // );

        for (int i = 0; i < 2 * N - 2; i++) {
            std::swap(u[i], u[i+2]);
        }
        u[2 * N - 2] = 0.0;
        u[2 * N - 1] = 0.0;

        pros::delay(10);
    }

    robot::chassis.brake();
}



// template <typename T>
// static T ad_motor_func(const int N, 
//     const std::vector<T>& x, 
//     const std::vector<DiffdriveState<var>>& desired,
//     const DiffdriveState<T>& state,
//     const DiffdriveMPCParams& params, 
//     const DiffdrivePenalty& penalty) 
// {
//     auto copy = state;
//     T cost = 0.0;
//     for (int i = 0; i < N; i++) {
//         T change = (params.gain * x[i] - copy.vl) / params.tc;
//         copy.vl += change * params.dt_guess / 1000.0f;
//         copy.y += copy.vl * (2 * M_PI / 60.0f) * (180 / M_PI) * params.dt_guess / 1000.0f;
//         copy.x += copy.y * params.dt_guess / 1000.0f;

//         const auto& dstate = desired[i];
//         const auto& error = copy - dstate;
//         cost += ((error * error * penalty).sum()) * get_scaling(params.scaling, i, N);
//         cost += 0.000 * change * change;
//         cost += 0.00002 * x[i] * x[i];
//     }
//     return cost;
// }

// template <int N>
// static double nlopt_motor_costfunc(unsigned int n, const double* x, double* grad, void* data) {
//     auto [desired, state, params, penality] = *static_cast<AggregateData*>(data);

//     std::vector<var> x_ad; 
//     for (int i = 0; i < n; i++) x_ad.push_back(x[i]);

//     var f = ad_motor_func<var>(N, x_ad, desired, state, params, penality);

//     if (grad) {
//         auto grad_ad = get_derivatives<N>(f, x_ad);
//         for (int i = 0; i < n; i++) grad[i] = autodiff::val(grad_ad[i]);
//     }

//     return autodiff::val(f);
// }

// static std::vector<DiffdriveState<var>> get_desired(const hardware::Motor& motor, int N, int dt) {
//     auto time = pros::millis();
//     std::vector<DiffdriveState<var>> desired;
//     for (float i = 1; i <= N; i++) {
//         desired.push_back({
//             sin((time + i * dt) / 1000.0f) * 100, i, i, i, i
//         });
//     }
//     return desired;
// }

// template <int N>
// void movement::simple::test_motor(
//     hardware::Motor& motor,      
//     const DiffdrivePenalty &penalty) 
// {
//     auto abs_start = pros::millis();
//     std::vector<double> x(N, 0.0);
//     DiffdriveMPCParams params {
//         .track_width = 1,
//         .gain = 1 / motor.get_velo_controller()->get_args().ka,
//         .tc = motor.get_velo_controller()->get_args().kv / motor.get_velo_controller()->get_args().ka
//     };
//     params.alg = nlopt::algorithm::LD_LBFGS;
//     params.scaling = MPCScaling::EXPONENTIAL;
//     params.ftol_rel = 0.0001;

//     float sum = 0;

//     while (true) {
//         auto start_t = pros::millis();
//         std::vector<DiffdriveState<var>> desired = get_desired(motor, N, params.optimization_dt);
//         DiffdriveState<var> state {
//             sum,
//             motor.get_position_average(),
//             motor.get_position_average(),
//             motor.get_filtered_velocity(),
//             motor.get_filtered_velocity()
//         };

//         AggregateData data {desired, state, params, penalty};

//         nlopt::opt opt(params.alg, N);
//         opt.set_ftol_rel(params.ftol_rel);
//         std::vector<double> lb(N, -12);
//         opt.set_lower_bounds(lb);
//         std::vector<double> ub(N, 12);
//         opt.set_upper_bounds(ub);
//         opt.set
//         opt.set_min_objective(nlopt_motor_costfunc<N>, &data);

//         double minf;
//         nlopt::result result = opt.optimize(x, minf);
//         if (result < 0) {
//             std::cerr << "NLOPT failed with code " << result << std::endl;
//         }

//         motor.set_desired_voltage(x[0] * 500 + x[1] * 250 + x[2] * 125);

//         for (int i = 0; i < N-1; i++) {
//             std::swap(x[i], x[i+1]);
//         }
//         x[N-1] = 0.0;

//         // pros::Task::delay_until(&start_t, params.dt + 10);
//         pros::delay(10);
//         params.dt = pros::millis() - start_t;
//         sum += motor.get_position_average() * params.dt / 1000.0f;
//         printf("%f,%f,%f,%f,%f\n", 
//             (pros::millis() - abs_start) / 1e3f, x[0], motor.get_position_average(), sum, params.dt / 1e3f);
//     }
// }

// volatile auto func_instance_3 = movement::simple::test_motor<3>;
// volatile auto func_instance_5 = movement::simple::test_motor<5>;
// volatile auto func_instance_7 = movement::simple::test_motor<7>;
// volatile auto func_instance_10 = movement::simple::test_motor<10>;
// volatile auto func_instance_15 = movement::simple::test_motor<15>;

volatile auto func_instance_3 = movement::simple::follow_recording<3>;
volatile auto func_instance_4 = movement::simple::follow_recording<4>;
volatile auto func_instance_5 = movement::simple::follow_recording<5>;
volatile auto func_instance_7 = movement::simple::follow_recording<7>;
volatile auto func_instance_10 = movement::simple::follow_recording<10>;
volatile auto func_instance_15 = movement::simple::follow_recording<15>;
#include "autonomous/movement/simple/mpc.h"

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
        std::forward<Func>(f), x_ad, std::make_index_sequence<2*N>{}
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
        copy.vl += (params.gain * x[2*i] - copy.vl) / params.tc * params.dt;
        copy.vr += (params.gain * x[2*i+1] - copy.vr) / params.tc * params.dt;
        T v = (copy.vl + copy.vr) / 2.0;
        T w = (copy.vl - copy.vr) / params.track_width;
        T half = w * params.dt / 2.0, travel = v * params.dt;
        T s = sinc(half);
        copy.rx += travel * s * sin(copy.theta + half);
        copy.ry += travel * s * cos(copy.theta + half);
        copy.theta += half * 2.0;

        const auto& dstate = desired[i];
        const auto& error = copy - dstate;
        cost += (error * error * penalty).sum() * get_scaling(params.scaling, i, N);
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

template <typename T>
static T ad_motor_func(const int N, 
    const std::vector<T>& x, 
    const std::vector<DiffdriveState<var>>& desired,
    const DiffdriveState<T>& state,
    const DiffdriveMPCParams& params, 
    const DiffdrivePenalty& penalty) 
{
    auto copy = state;
    T cost = 0.0;
    for (int i = 0; i < N; i++) {
        copy.vl += (params.gain * x[2*i] - copy.vl) / params.tc * params.dt / 1000.0f;
        copy.x += copy.vl * params.dt / 1000.0f;

        const auto& dstate = desired[i];
        const auto& error = copy - dstate;
        cost += (error * error * penalty).sum() * get_scaling(params.scaling, i, N);
    }
    return cost;
}

template <int N>
static double nlopt_motor_costfunc(unsigned int n, const double* x, double* grad, void* data) {
    printf("Called\n");
    printf("%d passed\n", (int) (data));
    pros::delay(1000);
    auto [desired, state, params, penality] = *static_cast<AggregateData*>(data);
    printf("Called\n");
    return 0.0f;

    std::vector<var> x_ad; 
    for (int i = 0; i < n; i++) x_ad.push_back(x[i]);

    var f = ad_motor_func<var>(N, x_ad, desired, state, params, penality);

    if (grad) {
        auto grad_ad = get_derivatives<N>(f, x_ad);
        for (int i = 0; i < n; i++) grad[i] = autodiff::val(grad_ad[i]);
    }

    return autodiff::val(f);
}

static std::vector<DiffdriveState<var>> get_desired(const hardware::Motor& motor, int N, int dt) {
    auto time = pros::millis();
    std::vector<DiffdriveState<var>> desired;
    for (float i = 1; i <= N; i++) {
        desired.push_back({
            cos((i * dt + time) / 1e3f) * motor.get_max_speed(), i, i, i, i
        });
    }
    return desired;
}

template <int N>
void movement::simple::test_motor(
    hardware::Motor& motor,      
    const DiffdrivePenalty &penalty) 
{
    std::vector<double> x(2*N, 1.0);
    auto start = pros::millis();
    DiffdriveMPCParams params {
        .track_width = 1,
        .gain = 1 / motor.get_velo_controller()->get_args().ka,
        .tc = motor.get_velo_controller()->get_args().kv / motor.get_velo_controller()->get_args().ka
    };
    params.dt = 10;
    params.alg = nlopt::algorithm::LD_SLSQP;
    params.scaling = MPCScaling::LINEAR;
    params.ftol_rel = 0.01;
    params.max_time = 20;

    while (true) {
        std::vector<DiffdriveState<var>> desired = get_desired(motor, N, params.dt);
        DiffdriveState<var> state {
            motor.get_position_average(),
            motor.get_position_average(),
            motor.get_position_average(),
            motor.get_filtered_velocity(),
            motor.get_filtered_velocity()
        };
        printf("MOtor velo: %f\n", motor.get_filtered_velocity());
        for (int i = 0; i < N; i++) {
            printf("Desired: %f\n", autodiff::val(desired[i].x));
        }

        AggregateData data {desired, state, params, penalty};

        nlopt::opt opt(params.alg, 2*N);
        opt.set_maxtime(params.max_time);
        opt.set_ftol_rel(params.ftol_rel);
        opt.set_lower_bounds(std::vector<double> (2*N, -12));
        opt.set_upper_bounds(std::vector<double> (2*N, -12));
        opt.set_min_objective(nlopt_motor_costfunc<N>, &data);

        double minf;
        nlopt::result result = opt.optimize(x, minf);
        if (result < 0) {
            std::cerr << "NLOPT failed with code " << result << std::endl;
        }

        motor.set_desired_voltage(x[0] * 1000);

        pros::Task::delay_until(&start, params.dt);
        start = pros::millis();
    }
}

volatile auto func_instance = movement::simple::test_motor<5>;
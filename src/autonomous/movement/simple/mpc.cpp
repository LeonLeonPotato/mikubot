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
        T change = (params.gain * x[i] - copy.vl) / params.tc;
        copy.vl += change * params.dt / 1000.0f;
        copy.y += copy.vl * (2 * M_PI / 60.0f) * (180 / M_PI) * params.dt / 1000.0f;
        copy.x += copy.y * params.dt / 1000.0f;

        const auto& dstate = desired[i];
        const auto& error = copy - dstate;
        cost += ((error * error * penalty).sum()) * get_scaling(params.scaling, i, N);
        cost += 0.000 * change * change;
        cost += 0.00002 * x[i] * x[i];
    }
    return cost;
}

template <int N>
static double nlopt_motor_costfunc(unsigned int n, const double* x, double* grad, void* data) {
    auto [desired, state, params, penality] = *static_cast<AggregateData*>(data);

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
            sin((time + i * dt) / 1000.0f) * 100, i, i, i, i
        });
    }
    return desired;
}

template <int N>
void movement::simple::test_motor(
    hardware::Motor& motor,      
    const DiffdrivePenalty &penalty) 
{
    auto abs_start = pros::millis();
    std::vector<double> x(N, 0.0);
    DiffdriveMPCParams params {
        .track_width = 1,
        .gain = 1 / motor.get_velo_controller()->get_args().ka,
        .tc = motor.get_velo_controller()->get_args().kv / motor.get_velo_controller()->get_args().ka
    };
    params.dt = 25;
    params.alg = nlopt::algorithm::LD_LBFGS;
    params.scaling = MPCScaling::EXPONENTIAL;
    params.ftol_rel = 0.0001;
    params.max_time = 20;

    float sum = 0;

    while (true) {
        auto start_t = pros::millis();
        std::vector<DiffdriveState<var>> desired = get_desired(motor, N, params.dt);
        DiffdriveState<var> state {
            sum,
            motor.get_position_average(),
            motor.get_position_average(),
            motor.get_filtered_velocity(),
            motor.get_filtered_velocity()
        };

        AggregateData data {desired, state, params, penalty};

        nlopt::opt opt(params.alg, N);
        opt.set_maxtime(params.max_time);
        opt.set_ftol_rel(params.ftol_rel);
        std::vector<double> lb(N, -12);
        opt.set_lower_bounds(lb);
        std::vector<double> ub(N, 12);
        opt.set_upper_bounds(ub);
        opt.set_min_objective(nlopt_motor_costfunc<N>, &data);

        double minf;
        nlopt::result result = opt.optimize(x, minf);
        if (result < 0) {
            std::cerr << "NLOPT failed with code " << result << std::endl;
        }

        motor.set_desired_voltage(x[0] * 500 + x[1] * 250 + x[2] * 125);

        for (int i = 0; i < N-1; i++) {
            std::swap(x[i], x[i+1]);
        }
        x[N-1] = 0.0;

        // pros::Task::delay_until(&start_t, params.dt + 10);
        pros::delay(10);
        params.dt = pros::millis() - start_t;
        sum += motor.get_position_average() * params.dt / 1000.0f;
        printf("%f,%f,%f,%f,%f\n", 
            (pros::millis() - abs_start) / 1e3f, x[0], motor.get_position_average(), sum, params.dt / 1e3f);
    }
}

volatile auto func_instance_3 = movement::simple::test_motor<3>;
volatile auto func_instance_5 = movement::simple::test_motor<5>;
volatile auto func_instance_7 = movement::simple::test_motor<7>;
volatile auto func_instance_10 = movement::simple::test_motor<10>;
volatile auto func_instance_15 = movement::simple::test_motor<15>;
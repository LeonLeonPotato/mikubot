#include "autonomous/movement/simple/mpc.h"

#include "nlopt/nlopt.hpp"
#include "autodiff/reverse/var.hpp"

using var = autodiff::Variable<double>;
using namespace movement::simple;

struct AggregateData {
    const DiffdriveMPCParams& params;
    const Penalty& penalty;
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
            return exp(i + 1) / exp(N);
        default:
            return 1.0;
    }
}

static var ad_diffdrive_costfunc(const int N, const std::vector<var>& x, const DiffdriveMPCParams& params, const Penalty& penalty) {
    var rx = 0.0, ry = 0.0, theta = 0.0, vl = 0.0, vr = 0.0;
    var cost = 0.0;
    for (int i = 0; i < N; i++) {
        vl += (params.gain * x[2*i] - vl) / params.tc * params.dt;
        vr += (params.gain * x[2*i+1] - vr) / params.tc * params.dt;
        var v = (vl + vr) / 2.0;
        var w = (vl - vr) / params.track_width;
        var half = w * params.dt / 2.0, travel = v * params.dt;
        var s = sinc(half);
        rx += travel * s * sin(theta + half);
        ry += travel * s * cos(theta + half);
        theta += half * 2.0;
    
        float scale = 1.0;
        switch (params.scaling) {
            case MPCScaling::LINEAR:
                scale = (float) (i + 1) / N;
                break;
            case MPCScaling::QUADRATIC:
                scale = pow(i + 1, 2) / pow(N, 2);
                break;
            case MPCScaling::EXPONENTIAL:
                scale = exp(i + 1) / exp(N);
                break;
            default:
                break;
        }
        cost += (penalty.x * rx * rx 
            + penalty.y * ry * ry 
            + penalty.theta * theta * theta 
            + penalty.vl * vl * vl 
            + penalty.vr * vr * vr) * scale;
    }
    return cost;
}

template <int N>
double nlopt_diffdrive_costfunc(unsigned int n, const double* x, double* grad, void* data) {
    auto& [params, penality] = *static_cast<AggregateData*>(data);

    std::vector<var> x_ad; 
    for (int i = 0; i < n; i++) x_ad.push_back(x[i]);

	var f = ad_diffdrive_costfunc(x_ad, params, penality);

	if (grad) {
        auto grad_ad = get_derivatives<N>(f, x_ad);
        for (int i = 0; i < n; i++) grad[i] = autodiff::val(grad_ad[i]);
    }

	return autodiff::val(f);
}

static var ad_diffdrive_costfunc(const std::vector<var>& x, const DiffdriveMPCParams& params, const Penalty& penalty) {
    var rx = 0.0, ry = 0.0, theta = 0.0, vl = 0.0, vr = 0.0;
    var cost = 0.0;
    for (int i = 0; i < params.N; i++) {
        vl += (params.gain * x[2*i] - vl) / params.tc * params.dt;
        vr += (params.gain * x[2*i+1] - vr) / params.tc * params.dt;
        var v = (vl + vr) / 2.0;
        var w = (vl - vr) / params.track_width;
        var half = w * params.dt / 2.0, travel = v * params.dt;
        var s = sinc(half);
        rx += travel * s * sin(theta + half);
        ry += travel * s * cos(theta + half);
        theta += half * 2.0;

        cost += (penalty.x * rx * rx 
            + penalty.y * ry * ry 
            + penalty.theta * theta * theta 
            + penalty.vl * vl * vl 
            + penalty.vr * vr * vr) * get_scaling(params.scaling, i, N);
    }
    return cost;
}
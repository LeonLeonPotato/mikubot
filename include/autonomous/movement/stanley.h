// #pragma once

// #include "autonomous/pathing/base_path.h"
// #include "autonomous/controllers.h"

// namespace movement {

// class Stanley : public BaseMovement {
//     protected:
//         float deriv(float t) const override;

//     public:
//         controllers::PID lateral_pid;

//         Stanley(pathing::BasePath& path, pathing::BaseParams& solve_params, 
//             const controllers::PID& angle_pid, const controllers::PID& lateral_pid) :
//             BaseMovement(path, solve_params, angle_pid), lateral_pid(lateral_pid) {}
//         Stanley(pathing::BasePath& path, pathing::BaseParams& solve_params) :
//             BaseMovement(path, solve_params) { set_generic_lateral_pid(); }

//         TickResult tick(float t) override;
//         MovementResult follow_path_cancellable(bool& cancel_ref) override;

//         void set_generic_lateral_pid(void);
//         static void init_generic_lateral_pid(controllers::PID& pid);
// };
// }
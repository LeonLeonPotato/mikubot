#include "opcontrol/driving.h"
#include "essential.h"
#include "api.h"
#include "nlohmann/json.h"

#include <iostream>

namespace driving {
struct LogObject {
	float x, y, theta;
	int left_voltage, right_voltage;
	float left_velo, right_velo;
	float left_accel, right_accel;
	float left_eff, right_eff;
	int dt;
};

std::vector<LogObject> logs;

inline void leon_mode(int left_x, int left_y, int right_x, int right_y) {
	if (abs(right_x) > 10 && abs(left_y) > 10) { // driving with turning
		int left = left_y + right_x;
		int right = left_y - right_x;
		robot::volt(left, right);
	} else if (abs(right_x) > 10 && abs(left_y) < 10) { // turning
		robot::volt(right_x, -right_x);
	} else if (abs(right_x) < 10 && abs(left_y) > 10) { // driving
		robot::volt(left_y, left_y);
	} else { // stop
		robot::brake();
	}
}

inline void leon_mode_velocity_based(int left_x, int left_y, int right_x, int right_y) {
	if (abs(right_x) > 10 && abs(left_y) > 10) { // driving with turning
		int left = left_y + right_x;
		int right = left_y - right_x;
		robot::velo(left, right);
	} else if (abs(right_x) > 10 && abs(left_y) < 10) { // turning
		robot::velo(right_x, -right_x);
	} else if (abs(right_x) < 10 && abs(left_y) > 10) { // driving
		robot::velo(left_y, left_y);
	} else { // stop
		robot::brake();
	}
}

inline void tank_drive(int left_x, int left_y, int right_x, int right_y) {
	if (std::min(abs(left_y), abs(right_y)) > 3) {
		robot::volt(left_y, right_y);
	} else {
		robot::brake();
	}
}

inline void tank_drive_velocity_based(int left_x, int left_y, int right_x, int right_y) {
	if (std::min(abs(left_y), abs(right_y)) > 3) {
		robot::velo(left_y, right_y);
	} else {
		robot::brake();
	}
}

int get_log_num(void) {
	FILE* fp = fopen("/usd/lognum.txt", "r");
	int num = 0;
	if (fp != NULL) {
		fscanf(fp, "%d", &num);
	}
	fclose(fp);
	fp = fopen("/usd/lognum.txt", "w");
	fprintf(fp, "%d", num + 1);
	fclose(fp);
	return num;
}

void print_logs(void) {
	int num = get_log_num();
	auto str = "/usd/driving_logs_" + std::to_string(num) + ".csv";

	FILE* fp = fopen(str.c_str(), "w");
	fprintf(fp, "x,y,theta,left_voltage,right_voltage,left_velo,right_velo,left_accel,right_accel,left_eff,right_eff,dt\n");

	int idx = 0;

	while (!pros::Task::notify_take(true, 50)) {
		for (; idx < logs.size(); idx++) {
			auto& log = logs[idx];
			fprintf(fp, 
				"%f,%f,%f,%d,%d,%f,%f,%f,%f,%f,%f,%d\n", 
				log.x, log.y, log.theta,
				log.left_voltage, log.right_voltage,
				log.left_velo, log.right_velo,
				log.left_accel, log.right_accel,
				log.left_eff, log.right_eff,
				log.dt
			);
		}
	}

	fclose(fp);
}

void run() {
	int it = 0;
	// pros::Task::create(print_logs);
	float left_last_velo = robot::left_motors.get_actual_velocity();
	float right_last_velo = robot::right_motors.get_actual_velocity();
	long long cur_time = pros::micros();

	while (true) {
		int left_x = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        int left_y = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int right_x = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int right_y = robot::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

		// swap out for other modes
		leon_mode(left_x, left_y, right_x, right_y);
		it++;

		long long dt = pros::micros() - cur_time;
		cur_time = pros::micros();

		float cur_left_velo = robot::left_motors.get_actual_velocity();
		float cur_right_velo = robot::right_motors.get_actual_velocity();
		float left_accel = (cur_left_velo - left_last_velo) / (dt / 1000000.0);
		float right_accel = (cur_right_velo - right_last_velo) / (dt / 1000000.0);
		logs.emplace_back(robot::x, robot::y, robot::theta,
							robot::left_motors.get_voltage(), robot::right_motors.get_voltage(),
							cur_left_velo, cur_right_velo,
							left_accel, right_accel,
							robot::left_motors.get_efficiency(), robot::right_motors.get_efficiency(),
							(int) dt);

		pros::delay(20);
	}
}
} // namespace driving
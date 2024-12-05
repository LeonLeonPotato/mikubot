#include "autonomous/strategy/playback.h"
#include "essential.h"
#include "config.h"

#include "api.h"

using namespace strategies;

static std::vector<std::string> tokenize(const char* buffer, size_t size) {
    std::vector<std::string> tokens;
    std::string token;
    for (size_t i = 0; i < size; i++) {
        if (*(buffer + i) == ',') {
            tokens.push_back(token);
            token.clear();
        } else {
            token.push_back(*(buffer + i));
        }
    }
    if (!token.empty()) tokens.push_back(token);

    return tokens;
}

static void play_strategy(const std::string& filename) {
    FILE* file = fopen(filename.c_str(), "r");
    if (!file) {
        printf("[Playback] Could not open file %s\n", filename.c_str());
        return;
    }

    char buffer[256]; fgets(buffer, 256, file);
    fread(buffer, 256, 1, file);
    while (fgets(buffer, 256, file)) {
        printf("%s", buffer);
    }

    while (true) {
        memset(buffer, 0, 256);
        if (fgets(buffer, 256, file) == NULL) break;
        
        std::string line(buffer);
        std::vector<std::string> tokens = tokenize(buffer, line.size());

        int left_set_velo = (int) std::stof(tokens[4]);
        int right_set_velo = (int) std::stof(tokens[5]);

        robot::left_motors.move_velocity(left_set_velo);
        robot::right_motors.move_velocity(right_set_velo);

        pros::delay(10);
    }
}

void playback::run(void) {
    // Do nothing for now
}
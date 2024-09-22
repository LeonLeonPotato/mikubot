#ifndef _MIKUBOT_AUTONOMOUS_STRATEGIES_H_
#define _MIKUBOT_AUTONOMOUS_STRATEGIES_H_

#include <unordered_map>
#include <string>

namespace strategies {
    enum class Strategy {
        Playback,
        Skills,
        TEST_1
    };

    extern const std::unordered_map<Strategy, std::string> names;
    extern const std::unordered_map<std::string, Strategy> values;
    extern const Strategy default_strategy;
}

#endif

#pragma once

#include <string>

namespace debugscreen {
extern std::string debug_message;

void init(void);
void destroy(void);
} // namespace debugscreen
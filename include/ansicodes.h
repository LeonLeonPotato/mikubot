#pragma once

#include <string>

constexpr std::string ANSI_RESET = "\x1b[0m";
constexpr std::string ANSI_BOLD = "\x1b[1m";
constexpr std::string ANSI_DIM = "\x1b[2m";
constexpr std::string ANSI_ITALIC = "\x1b[3m";
constexpr std::string ANSI_UNDERLINE = "\x1b[4m";
constexpr std::string ANSI_BLINK = "\x1b[5m";
constexpr std::string ANSI_REVERSE = "\x1b[7m";
constexpr std::string ANSI_INVISIBLE = "\x1b[8m";

constexpr std::string ANSI_BLACK = "\x1b[30m";
constexpr std::string ANSI_RED = "\x1b[31m";
constexpr std::string ANSI_GREEN = "\x1b[32m";
constexpr std::string ANSI_YELLOW = "\x1b[33m";
constexpr std::string ANSI_BLUE = "\x1b[34m";
constexpr std::string ANSI_MAGENTA = "\x1b[35m";
constexpr std::string ANSI_CYAN = "\x1b[36m";
constexpr std::string ANSI_WHITE = "\x1b[37m";

static const auto PREFIX = ANSI_BOLD + ANSI_CYAN + "[Miku" + ANSI_GREEN + "bot] " + ANSI_RESET;
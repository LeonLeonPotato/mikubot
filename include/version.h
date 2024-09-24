#pragma once

#define VERSION_MAJOR 1
#define VERSION_MINOR 1
#define VERSION_PATCH 0
#define VERSION_RELEASE "alpha"

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#define VERSION_STRING VERSION_RELEASE "-" \
                        TOSTRING(VERSION_MAJOR) "." \
                        TOSTRING(VERSION_MINOR) "." \
                        TOSTRING(VERSION_PATCH)
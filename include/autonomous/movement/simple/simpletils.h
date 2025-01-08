#pragma once

#include "autonomous/future.h"
#include "autonomous/movement/base_movement.h"

#define DECLARE_CANCELLABLE(func, pidtype, ...)                  \
SimpleResult func##_cancellable(                                 \
    __VA_ARGS__,                                                 \
    const SimpleMovementParams& params,                          \
    pidtype pids,                                                \
    volatile bool& cancel_ref                                    \
)

#define DEFINE_CANCELLABLE(func, pidtype, ...)                   \
SimpleResult simple::func##_cancellable(                         \
    __VA_ARGS__,                                                 \
    const SimpleMovementParams& params,                          \
    pidtype pids,                                                \
    volatile bool& cancel_ref                                    \
)

#define DECLARE_ASYNC(func, pidtype, ...)                        \
Future<SimpleResult> func##_async(                               \
    __VA_ARGS__,                                                 \
    const SimpleMovementParams& params,                          \
    pidtype pids                                                 \
)

#define DEFINE_ASYNC(func, pidtype, ...)                         \
Future<SimpleResult> simple::func##_async(                       \
    __VA_ARGS__,                                                 \
    const SimpleMovementParams& params,                          \
    pidtype pids                                                 \
)

#define DECLARE_STANDARD(func, pidtype, ...)                     \
SimpleResult func(                                               \
    __VA_ARGS__,                                                 \
    const SimpleMovementParams& params,                          \
    pidtype pids                                                 \
)

#define DEFINE_STANDARD(func, pidtype, ...)                      \
SimpleResult simple::func(                                       \
    __VA_ARGS__,                                                 \
    const SimpleMovementParams& params,                          \
    pidtype pids                                                 \
)

#define DECLARE_TICK(func, pidtype, ...)                         \
SimpleResult func##_tick(                                        \
    __VA_ARGS__,                                                 \
    const SimpleMovementParams& params,                          \
    pidtype pids                                                 \
)

#define DEFINE_TICK(func, pidtype, ...)                          \
SimpleResult simple::func##_tick(                                \
    __VA_ARGS__,                                                 \
    const SimpleMovementParams& params,                          \
    pidtype pids                                                 \
)

// this shit fucking gives newline errors if i do backslash for some reason
// so this is the only solution i guess
#define DECLARE_ALL(func, pidtype, ...) DECLARE_TICK(func, pidtype, __VA_ARGS__); DECLARE_CANCELLABLE(func, pidtype, __VA_ARGS__); DECLARE_STANDARD(func, pidtype, __VA_ARGS__); DECLARE_ASYNC(func, pidtype, __VA_ARGS__);
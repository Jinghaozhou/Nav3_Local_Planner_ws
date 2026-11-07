#pragma once
#include <cstdio>
// Kept independent of ROS; the wrapper reports the final planning failure.
#define SCAN_LOG(...) do { std::fprintf(stderr, __VA_ARGS__); std::fputc('\n', stderr); } while (false)

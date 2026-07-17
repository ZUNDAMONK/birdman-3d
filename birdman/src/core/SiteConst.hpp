#pragma once

#include <array>
#include <cmath>

namespace bm::site {

// 富士川滑空場の描画・物理で共有する唯一の寸法定義。
inline constexpr double FUJI_RWY_LENGTH = 850.0;
inline constexpr double FUJI_RWY_HALF_WIDTH = 15.0;
inline constexpr double FUJI_RWY_SOUTH_Z = 10.0;
inline constexpr double FUJI_RWY_NORTH_Z = 860.0;
inline constexpr double FUJI_START_Z_BASE = 850.0;
inline constexpr double FUJI_START_POS_MIN = 0.0;
inline constexpr double FUJI_START_POS_MAX = 840.0;
inline constexpr double FUJI_START_POS_DEFAULT = 830.0;
inline constexpr double FUJI_OVERRUN_DISTANCE = 1050.0;

inline constexpr double FUJI_RWY_MU = 0.02;
inline constexpr double FUJI_RIVERBED_MU = 0.05;
inline constexpr double FUJI_SANDBAR_MU = 0.08;
inline constexpr double FUJI_BROKEN_GEAR_MU = 0.15;

inline constexpr double FUJI_RIVER_UPSTREAM_X = -8500.0;
inline constexpr double FUJI_RIVER_MOUTH_NEAR_X = 1380.0;
inline constexpr double FUJI_SEA_START_X = 1400.0;
inline constexpr double FUJI_RIVER_NEAR_BANK = -45.0;
inline constexpr double FUJI_RIVER_FAR_BANK_UPSTREAM = -255.0;
inline constexpr double FUJI_RIVER_FAR_BANK_MOUTH = -700.0;
inline constexpr double FUJI_RIVER_WIDEN_START_X = -200.0;
inline constexpr double FUJI_RIVER_WIDEN_END_X = 1300.0;

struct Sandbar {
    double ylCenter;
    double courseXCenter;
    double ylWidth;
    double courseLength;
};

inline constexpr std::array<Sandbar, 9> FUJI_SANDBARS = {{
    {-160.0, -120.0, 50.0, 64.0},
    {-173.0, -300.0, 65.0, 64.0},
    {-161.0, -470.0, 70.0, 64.0},
    {-174.0, -650.0, 62.0, 64.0},
    {-162.0, -820.0, 46.0, 64.0},
    {-180.0,  750.0, 60.0, 100.0},
    {-320.0,  950.0, 80.0, 120.0},
    {-220.0, 1120.0, 55.0, 85.0},
    {-380.0, 1220.0, 70.0, 95.0},
}};

inline constexpr double fujikawaFarBank(double courseX) {
    if (courseX <= FUJI_RIVER_WIDEN_START_X) return FUJI_RIVER_FAR_BANK_UPSTREAM;
    if (courseX >= FUJI_RIVER_WIDEN_END_X) return FUJI_RIVER_FAR_BANK_MOUTH;
    const double u = (courseX - FUJI_RIVER_WIDEN_START_X)
                   / (FUJI_RIVER_WIDEN_END_X - FUJI_RIVER_WIDEN_START_X);
    return FUJI_RIVER_FAR_BANK_UPSTREAM
         + (FUJI_RIVER_FAR_BANK_MOUTH - FUJI_RIVER_FAR_BANK_UPSTREAM) * u;
}

inline bool insideFujikawaRunway(double courseX, double yl) {
    return std::abs(yl) <= FUJI_RWY_HALF_WIDTH
        && courseX >= -FUJI_RWY_NORTH_Z
        && courseX <= -FUJI_RWY_SOUTH_Z;
}

inline bool insideFujikawaSandbar(double courseX, double yl) {
    for (const Sandbar& bar : FUJI_SANDBARS) {
        if (std::abs(courseX - bar.courseXCenter) <= bar.courseLength * 0.5
            && std::abs(yl - bar.ylCenter) <= bar.ylWidth * 0.5)
            return true;
    }
    return false;
}

inline double fujikawaStartWorldZ(double startPos) {
    if (startPos < FUJI_START_POS_MIN) startPos = FUJI_START_POS_MIN;
    if (startPos > FUJI_START_POS_MAX) startPos = FUJI_START_POS_MAX;
    return FUJI_START_Z_BASE - startPos;
}

} // namespace bm::site

#pragma once

namespace fast_lio
{
    namespace constants
    {
        // Mathematical constants
        inline constexpr double PI = 3.14159265358;
        inline constexpr double PI_M = PI; // Backward compatibility

        // Physical constants
        inline constexpr double G_m_s2 = 9.81; // Gravity in GuangDong/China

        // System dimensions
        inline constexpr int DIM_STATE = 18;  // Dimension of states (Let Dim(SO(3)) = 3)
        inline constexpr int DIM_PROC_N = 12; // Dimension of process noise

        // Algorithm parameters
        inline constexpr double CUBE_LEN = 6.0;
        inline constexpr int LIDAR_SP_LEN = 2;
        inline constexpr double INIT_COV = 1.0;
        inline constexpr int NUM_MATCH_POINTS = 5;
        inline constexpr int MAX_MEAS_DIM = 10000;

        // Array size limits
        inline constexpr int MAXN = 720000;
    }
} // namespace fast_lio
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// UKF shadow estimator: SITL and Replay only
#ifndef HAL_NAVUKF_AVAILABLE
#define HAL_NAVUKF_AVAILABLE (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif
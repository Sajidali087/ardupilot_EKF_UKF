#pragma once

#include "AP_NavUKF_config.h"

#if HAL_NAVUKF_AVAILABLE

#include <AP_Param/AP_Param.h>

class NavUKF {
public:
    NavUKF();

    CLASS_NO_COPY(NavUKF);

    static const struct AP_Param::GroupInfo var_info[];

    bool enabled() const { return _enable != 0; }

    // run one filter step; called by AP_AHRS after EKF3 and by Replay from the RFRF frame
    void UpdateFilter(void);

    // number of UpdateFilter calls that ran (stub check for Replay)
    uint32_t frame_count() const { return _frame_count; }

private:
    AP_Int8 _enable;

    uint32_t _frame_count;
};

#endif // HAL_NAVUKF_AVAILABLE

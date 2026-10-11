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

private:
    AP_Int8 _enable;
};

#endif // HAL_NAVUKF_AVAILABLE
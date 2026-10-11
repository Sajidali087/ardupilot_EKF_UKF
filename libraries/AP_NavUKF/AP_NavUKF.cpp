#include "AP_NavUKF.h"

#if HAL_NAVUKF_AVAILABLE

const AP_Param::GroupInfo NavUKF::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Enable UKF shadow estimator
    // @Description: Runs the UKF next to EKF3 in SITL and Replay. It never controls the vehicle.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 0, NavUKF, _enable, 0, AP_PARAM_FLAG_ENABLE),

    AP_GROUPEND
};

NavUKF::NavUKF()
{
    AP_Param::setup_object_defaults(this, var_info);
}

#endif // HAL_NAVUKF_AVAILABLE
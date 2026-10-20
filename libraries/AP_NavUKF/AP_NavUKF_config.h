/*
  UKF shadow estimator for ArduPilot (libraries/AP_NavUKF): build configuration

  Copyright (C) 2026 the Client, the party that commissioned this work (see NOTICE in this directory).

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// UKF shadow estimator: SITL and Replay only
#ifndef HAL_NAVUKF_AVAILABLE
#define HAL_NAVUKF_AVAILABLE (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

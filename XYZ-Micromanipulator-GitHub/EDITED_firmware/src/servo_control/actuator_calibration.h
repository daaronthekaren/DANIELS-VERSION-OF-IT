// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------

#pragma once

#include "servo_controller.h"
#include "utilities/lookup_table.h"

//*** FUNCTIONS *************************************************************************

bool measure_calibration_data(
  LookupTable& encoder_raw_to_motor_pos_lut,
  LookupTable& motor_pos_to_field_angle_lut,
  ServoController& servo_controller,
  float field_angle_range,
  float field_velocity,
  size_t size,
  bool print_measurements);


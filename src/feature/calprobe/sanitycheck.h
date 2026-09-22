/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 Alberto Cotronei @MagoKimbra
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */
#pragma once

/**
 * sanitycheck.h
 *
 * Test configuration values for errors at compile-time.
 */

#if ENABLED(CALIBRATION_PROBE)
  #if !HAS_Z_PROBE_PIN
    #error "DEPENDENCY ERROR: CALIBRATION_PROBE requires Z_PROBE_PIN."
  #endif
  #if HAS_BED_PROBE
    #error "CONFLICT ERROR: CALIBRATION_PROBE uses Z_PROBE_PIN and can't be used with a bed probe."
  #endif
  #if IS_KINEMATIC
    #error "DEPENDENCY ERROR: CALIBRATION_PROBE requires Cartesian or Core mechanics."
  #endif
  #if DISABLED(CALIBRATION_PROBE_SPREAD) || DISABLED(CALIBRATION_PROBE_LOWER_Z) || DISABLED(CALIBRATION_PROBE_LIFT_Z) \
   || DISABLED(CALIBRATION_PROBE_SEARCH_Z) || DISABLED(CALIBRATION_PROBE_FINAL_LIFT_Z) || DISABLED(CALIBRATION_PROBE_RETRACT) \
   || DISABLED(CALIBRATION_PROBE_SAMPLES) || DISABLED(CALIBRATION_PROBE_TOLERANCE) \
   || DISABLED(CALIBRATION_PROBE_FEEDRATE_FAST) || DISABLED(CALIBRATION_PROBE_FEEDRATE_SLOW) \
   || DISABLED(CALIBRATION_PROBE_FEEDRATE_TRAVEL) || DISABLED(CALIBRATION_PROBE_FEEDRATE_Z)
    #error "DEPENDENCY ERROR: Missing CALIBRATION_PROBE_* setting, see Configuration_Feature.h."
  #endif
#endif

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

/**
 * calprobe.cpp - XYZ nozzle offset calibration probe
 */

#include "../../../MK4duo.h"
#include "sanitycheck.h"

#if HAS_CALIBRATION_PROBE

CalibrationProbe calprobe;

/** Public Parameters */
float   CalibrationProbe::spread          = CALIBRATION_PROBE_SPREAD,
        CalibrationProbe::lower_z         = CALIBRATION_PROBE_LOWER_Z,
        CalibrationProbe::lift_z          = CALIBRATION_PROBE_LIFT_Z,
        CalibrationProbe::tolerance       = CALIBRATION_PROBE_TOLERANCE,
        CalibrationProbe::feedrate_slow   = CALIBRATION_PROBE_FEEDRATE_SLOW;
uint8_t CalibrationProbe::samples         = CALIBRATION_PROBE_SAMPLES;

bool    CalibrationProbe::located         = false;
uint8_t CalibrationProbe::located_tool    = 0;
float   CalibrationProbe::center[XYZ]     = { 0.0 },
        CalibrationProbe::width[2]        = { 0.0 },
        CalibrationProbe::range[XYZ]      = { 0.0 };

/** Public Function */

bool CalibrationProbe::is_triggered() {
  return READ(Z_PROBE_PIN) != endstops.isLogic(Z_PROBE);
}

CalibrationProbe::ProbeResult CalibrationProbe::probe_move(const AxisEnum axis, const float distance, const float fr_mm_s) {
  float target[XYZ] = { mechanics.current_position[X_AXIS], mechanics.current_position[Y_AXIS], mechanics.current_position[Z_AXIS] };
  target[axis] += distance;
  return guarded_move(target, fr_mm_s);
}

bool CalibrationProbe::release(const AxisEnum axis, const int8_t dir) {
  for (uint8_t i = 0; i < 4; i++) {
    float target[XYZ] = { mechanics.current_position[X_AXIS], mechanics.current_position[Y_AXIS], mechanics.current_position[Z_AXIS] };
    target[axis] -= dir * (CALIBRATION_PROBE_RETRACT);
    if (!reachable(target)) return false;
    mechanics.do_blocking_move_to(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], CALIBRATION_PROBE_FEEDRATE_FAST);
    printer.safe_delay(10); // Let the switch settle
    if (!is_triggered()) return true;
  }
  SERIAL_LM(ER, "Calibration probe is not released after the touch");
  return false;
}

bool CalibrationProbe::locate(const uint8_t verbose) {

  if (mechanics.axis_unhomed_error()) return false;

  stepper.synchronize();

  if (is_triggered()) {
    SERIAL_LM(ER, "Calibration probe is triggered, check the probe and its wiring (M119)");
    return false;
  }

  printer.setup_for_endstop_or_probe_move();
  const bool ok = do_locate(verbose);
  printer.clean_up_after_endstop_or_probe_move();
  return ok;
}

void CalibrationProbe::report() {
  if (located) {
    SERIAL_SMV(ECHO, "Calibration probe position T", (int)located_tool);
    SERIAL_MV(" X:", LOGICAL_X_POSITION(center[X_AXIS]), 3);
    SERIAL_MV(" Y:", LOGICAL_Y_POSITION(center[Y_AXIS]), 3);
    SERIAL_EMV(" Z:", LOGICAL_Z_POSITION(center[Z_AXIS]), 3);
  }
  else
    SERIAL_LM(ECHO, "Calibration probe position unknown");
}

/** Private Function */

/**
 * Locate the probe:
 *  - Coarse pass with single fast touches: find the probe top below the nozzle,
 *    then the center from the X and Y sides
 *  - Precise pass with several slow touches: the probe top above the found center,
 *    then the X and Y sides again at the precise height
 */
bool CalibrationProbe::do_locate(const uint8_t verbose) {

  // Start above the located probe position
  if (located) {
    if (!move_z(max(mechanics.current_position[Z_AXIS], center[Z_AXIS] + lift_z))) return false;
    if (!move_xy(center[X_AXIS], center[Y_AXIS])) return false;
    if (!move_z(center[Z_AXIS] + lift_z)) return false;
  }

  float c[XYZ] = { mechanics.current_position[X_AXIS], mechanics.current_position[Y_AXIS], 0.0 };

  // Coarse pass
  if (verbose) SERIAL_LM(ECHO, "Calibration probe coarse pass");

  const ProbeResult found = probe_move(Z_AXIS, -(CALIBRATION_PROBE_SEARCH_Z), CALIBRATION_PROBE_FEEDRATE_FAST);
  if (found == PROBE_NO_CONTACT) {
    SERIAL_LM(ER, "Calibration probe not found below the nozzle");
    if (located)
      SERIAL_LM(ECHO, "If the probe was moved, reset its position with M1020 R");
    else
      SERIAL_LM(ECHO, "Place the nozzle 2-3 mm above the probe center");
  }
  if (found != PROBE_CONTACT) return false;

  c[Z_AXIS] = mechanics.current_position[Z_AXIS];
  if (verbose > 1) report_touch(Z_AXIS, -1, c[Z_AXIS]);
  if (!release(Z_AXIS, -1)) return false;

  if (!probe_sides(c, 0, verbose)) return false;

  // Precise pass
  if (verbose) SERIAL_LM(ECHO, "Calibration probe precise pass");

  if (!move_xy(c[X_AXIS], c[Y_AXIS])) return false;
  if (!touch(Z_AXIS, -1, lift_z + lower_z, samples, c[Z_AXIS], range[Z_AXIS], verbose)) return false;
  if (verbose) {
    SERIAL_SMV(ECHO, "  Z top ", LOGICAL_Z_POSITION(c[Z_AXIS]), 3);
    SERIAL_EMV(" range ", range[Z_AXIS], 3);
  }

  if (!probe_sides(c, samples, verbose)) return false;

  // Park above the probe center
  if (!move_z(c[Z_AXIS] + CALIBRATION_PROBE_FINAL_LIFT_Z)) return false;
  if (!move_xy(c[X_AXIS], c[Y_AXIS])) return false;

  COPY_ARRAY(center, c);
  located = true;
  located_tool = tools.active_extruder;
  return true;
}

/**
 * Move to the target checking the probe, the move is stopped when the probe triggers.
 * The current position is updated to where the steppers stopped.
 */
CalibrationProbe::ProbeResult CalibrationProbe::guarded_move(const float target[XYZ], const float fr_mm_s) {

  // The probe must stop only this move, not the moves queued before
  stepper.synchronize();

  if (is_triggered()) {
    SERIAL_LM(ER, "Calibration probe is triggered before the move");
    return PROBE_FAILED;
  }

  if (!reachable(target)) return PROBE_FAILED;

  float dest[XYZE];
  COPY_ARRAY(dest, mechanics.current_position);
  AxisEnum axis = X_AXIS; // Main moving axis, the whole move is stopped anyway
  LOOP_XYZ(i) {
    dest[i] = target[i];
    if (FABS(dest[i] - mechanics.current_position[i]) > FABS(dest[axis] - mechanics.current_position[axis])) axis = (AxisEnum)i;
  }

  endstops.hit_on_purpose();
  endstops.setCalibrationProbe(axis);
  planner.buffer_line(dest, fr_mm_s, tools.active_extruder);
  stepper.synchronize();
  endstops.setCalibrationProbe(NO_AXIS);

  const char hits = endstops.hit_bits;
  endstops.hit_on_purpose();

  // Get the position where the steppers were stopped
  mechanics.set_current_from_steppers_for_axis(ALL_AXES);
  mechanics.sync_plan_position_mech_specific();

  // A contact at the very end of the move may be not debounced yet
  if (TEST(hits, Z_PROBE) || is_triggered()) return PROBE_CONTACT;

  if (hits) {
    SERIAL_LM(ER, "Calibration probe move stopped by an endstop");
    return PROBE_FAILED;
  }

  return PROBE_NO_CONTACT;
}

/**
 * Move around the probe, the probe must not be touched
 */
bool CalibrationProbe::travel(const float x, const float y, const float z, const float fr_mm_s) {
  const float target[XYZ] = { x, y, z };
  const ProbeResult result = guarded_move(target, fr_mm_s);
  if (result == PROBE_CONTACT) {
    SERIAL_LM(ER, "Calibration probe touched while moving around it");
    SERIAL_LM(ECHO, "Check the nozzle start position and the settings: D, L, H (M1020)");
    release(Z_AXIS, -1);
  }
  return result == PROBE_NO_CONTACT;
}

/**
 * Check that a position is inside the software endstops
 */
bool CalibrationProbe::reachable(const float target[XYZ]) {
  float clamped[XYZ] = { target[X_AXIS], target[Y_AXIS], target[Z_AXIS] };
  endstops.clamp_to_software_endstops(clamped);
  LOOP_XYZ(i) {
    if (clamped[i] != target[i]) {
      SERIAL_SM(ER, "Calibration probe move out of limits");
      SERIAL_MV(" X:", LOGICAL_X_POSITION(target[X_AXIS]), 3);
      SERIAL_MV(" Y:", LOGICAL_Y_POSITION(target[Y_AXIS]), 3);
      SERIAL_EMV(" Z:", LOGICAL_Z_POSITION(target[Z_AXIS]), 3);
      return false;
    }
  }
  return true;
}

bool CalibrationProbe::move_z(const float z) {
  return travel(mechanics.current_position[X_AXIS], mechanics.current_position[Y_AXIS], z, CALIBRATION_PROBE_FEEDRATE_Z);
}

bool CalibrationProbe::move_xy(const float x, const float y) {
  return travel(x, y, mechanics.current_position[Z_AXIS], CALIBRATION_PROBE_FEEDRATE_TRAVEL);
}

/**
 * Touch the probe moving along the axis in the given direction from the current position.
 * count = 0: a single fast touch. Otherwise a fast touch to find the probe and then
 * 'count' slow touches, contact is their average and spread_mm the difference between them.
 * The probe is released at the end.
 */
bool CalibrationProbe::touch(const AxisEnum axis, const int8_t dir, const float max_dist, const uint8_t count, float &contact, float &spread_mm, const uint8_t verbose) {

  ProbeResult result = probe_move(axis, dir * max_dist, CALIBRATION_PROBE_FEEDRATE_FAST);

  float pos = mechanics.current_position[axis], sum = 0.0, pmin = 0.0, pmax = 0.0;

  for (uint8_t s = 0; ; s++) {

    if (result == PROBE_NO_CONTACT) {
      SERIAL_SM(ER, "Calibration probe: no contact moving ");
      SERIAL_CHR(axis_codes[axis]);
      SERIAL_CHR(dir > 0 ? '+' : '-');
      SERIAL_EOL();
    }
    if (result != PROBE_CONTACT) return false;

    pos = mechanics.current_position[axis];
    if (verbose > 1) report_touch(axis, dir, pos);

    // The first touch of the precise pass only finds the probe, it is not counted
    if (s) {
      sum += pos;
      if (s == 1) pmin = pmax = pos;
      NOMORE(pmin, pos);
      NOLESS(pmax, pos);
    }

    if (!release(axis, dir)) return false;

    if (s >= count) break;

    // Approach again from the release position, a bit further than the last contact
    const float dist = FABS(mechanics.current_position[axis] - pos) + (CALIBRATION_PROBE_RETRACT);
    result = probe_move(axis, dir * dist, feedrate_slow);
  }

  if (!count) {
    contact = pos;
    spread_mm = 0.0;
    return true;
  }

  contact = sum / count;
  spread_mm = pmax - pmin;

  if (spread_mm > tolerance) {
    SERIAL_SM(ER, "Calibration probe: touches ");
    SERIAL_CHR(axis_codes[axis]);
    SERIAL_CHR(dir > 0 ? '+' : '-');
    SERIAL_MV(" differ by ", spread_mm, 3);
    SERIAL_EMV(" mm, tolerance ", tolerance, 3);
    return false;
  }

  return true;
}

/**
 * Touch the probe from X+, X-, Y+ and Y- at lower_z below the probe top c[Z]
 * and set c[X], c[Y] to the middle between the opposite contacts.
 * The Y sides are touched on the X center already found.
 */
bool CalibrationProbe::probe_sides(float c[XYZ], const uint8_t count, const uint8_t verbose) {

  const float travel_z  = c[Z_AXIS] + lift_z,
              side_z    = c[Z_AXIS] - lower_z;

  LOOP_S_LE_N(a, X_AXIS, Y_AXIS) {
    const AxisEnum axis = (AxisEnum)a;
    float contact[2], spread_mm[2];

    // contact[0] moving + (near side), contact[1] moving - (far side)
    for (uint8_t i = 0; i < 2; i++) {
      const int8_t dir = i ? -1 : 1;
      float start[XYZ] = { c[X_AXIS], c[Y_AXIS], side_z };
      start[axis] -= dir * spread;
      if (!move_z(travel_z)) return false;
      if (!move_xy(start[X_AXIS], start[Y_AXIS])) return false;
      if (!move_z(side_z)) return false;
      if (!touch(axis, dir, 2 * spread, count, contact[i], spread_mm[i], verbose)) return false;
    }

    width[axis] = contact[1] - contact[0];
    if (width[axis] <= 0.0) {
      SERIAL_SMV(ER, "Calibration probe: wrong contacts on ", axis_codes[axis]);
      SERIAL_MV(" ", mechanics.native_to_logical(contact[0], axis), 3);
      SERIAL_EMV(" ", mechanics.native_to_logical(contact[1], axis), 3);
      return false;
    }

    c[axis] = (contact[0] + contact[1]) * 0.5;
    range[axis] = max(spread_mm[0], spread_mm[1]);

    if (verbose) {
      SERIAL_SMV(ECHO, "  ", axis_codes[axis]);
      SERIAL_MV(" ", mechanics.native_to_logical(contact[0], axis), 3);
      SERIAL_MV(" .. ", mechanics.native_to_logical(contact[1], axis), 3);
      SERIAL_MV(" center ", mechanics.native_to_logical(c[axis], axis), 3);
      SERIAL_MV(" width ", width[axis], 3);
      SERIAL_EMV(" range ", range[axis], 3);
    }
  }

  return move_z(travel_z);
}

void CalibrationProbe::report_touch(const AxisEnum axis, const int8_t dir, const float pos) {
  SERIAL_SM(ECHO, "  touch ");
  SERIAL_CHR(axis_codes[axis]);
  SERIAL_CHR(dir > 0 ? '+' : '-');
  SERIAL_EMV(" ", mechanics.native_to_logical(pos, axis), 3);
}

#endif // HAS_CALIBRATION_PROBE

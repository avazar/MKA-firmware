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
 * mcode
 *
 * Nozzle calibration probe
 */

#if HAS_CALIBRATION_PROBE

  #define CODE_M1020
  #define CODE_M1021
  #define CODE_M1022

  /**
   * M1020: Calibration probe state and settings
   *
   *  M1020    - Report the probe state, the located probe position and the settings
   *  R        - Forget the located probe position (after the probe was moved)
   *
   *  Settings, kept until restart:
   *  D<mm>    - Side touches start distance from the probe center
   *  L<mm>    - Height of the side touches below the probe top
   *  H<mm>    - Travel height above the probe top
   *  S<count> - Touches per side in the precise pass
   *  T<mm>    - Max difference between the touches of one side
   *  F<mm/m>  - Speed of the precise touches
   */
  inline void gcode_M1020(void) {

    if (parser.seen('R')) calprobe.located = false;

    if (parser.seenval('D')) calprobe.spread        = constrain(parser.value_linear_units(), 1.0, 20.0);
    if (parser.seenval('L')) calprobe.lower_z       = constrain(parser.value_linear_units(), 0.1, 5.0);
    if (parser.seenval('H')) calprobe.lift_z        = constrain(parser.value_linear_units(), 0.5, 10.0);
    if (parser.seenval('S')) calprobe.samples       = constrain(parser.value_int(), 1, 10);
    if (parser.seenval('T')) calprobe.tolerance     = constrain(parser.value_linear_units(), 0.001, 1.0);
    if (parser.seenval('F')) calprobe.feedrate_slow = constrain(MMM_TO_MMS(parser.value_feedrate()), 0.1, 10.0);

    SERIAL_LMT(ECHO, "Calibration probe: ", calprobe.is_triggered() ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN);
    calprobe.report();
    SERIAL_SMV(ECHO, "Calibration probe settings: D", calprobe.spread, 2);
    SERIAL_MV(" L", calprobe.lower_z, 2);
    SERIAL_MV(" H", calprobe.lift_z, 2);
    SERIAL_MV(" S", (int)calprobe.samples);
    SERIAL_MV(" T", calprobe.tolerance, 3);
    SERIAL_EMV(" F", MMS_TO_MMM(calprobe.feedrate_slow), 0);
  }

  /**
   * M1021: Single probing move with the active nozzle
   *
   *  M1021 X<mm> | Y<mm> | Z<mm> [F<mm/m>]
   *
   *  Move along one axis by the given distance (relative) until the probe triggers,
   *  report the contact position and move back until the probe is released.
   *  F - Probing speed, the precise touches speed by default.
   */
  inline void gcode_M1021(void) {

    if (mechanics.axis_unhomed_error()) return;

    AxisEnum axis = NO_AXIS;
    float distance = 0.0;
    uint8_t axes = 0;
    LOOP_XYZ(i) {
      if (parser.seenval(axis_codes[i])) {
        axis = (AxisEnum)i;
        distance = parser.value_axis_units(axis);
        axes++;
      }
    }
    if (axes != 1 || distance == 0.0) {
      SERIAL_LM(ER, "M1021: set the distance on one axis: X, Y or Z");
      return;
    }

    const float fr_mm_s = parser.seenval('F') ? constrain(MMM_TO_MMS(parser.value_feedrate()), 0.1, 10.0) : calprobe.feedrate_slow;

    printer.setup_for_endstop_or_probe_move();

    const CalibrationProbe::ProbeResult result = calprobe.probe_move(axis, distance, fr_mm_s);
    if (result == CalibrationProbe::PROBE_CONTACT) {
      SERIAL_SM(ECHO, "Calibration probe contact");
      SERIAL_MV(" X:", LOGICAL_X_POSITION(mechanics.current_position[X_AXIS]), 3);
      SERIAL_MV(" Y:", LOGICAL_Y_POSITION(mechanics.current_position[Y_AXIS]), 3);
      SERIAL_EMV(" Z:", LOGICAL_Z_POSITION(mechanics.current_position[Z_AXIS]), 3);
      calprobe.release(axis, distance > 0.0 ? 1 : -1);
    }
    else if (result == CalibrationProbe::PROBE_NO_CONTACT)
      SERIAL_LM(ECHO, "Calibration probe: no contact");

    printer.clean_up_after_endstop_or_probe_move();
  }

  /**
   * M1022: Locate the calibration probe with the active nozzle
   *
   *  Starts above the located probe position. If the probe was not located yet
   *  (after restart or M1020 R), starts from the current position: place the nozzle
   *  2-3 mm above the probe center, within a few mm in X and Y.
   *  The nozzle is parked above the probe center at the end.
   *
   *  V<level> - 0 = result only, 1 = result of each side (default), 2 = every touch
   */
  inline void gcode_M1022(void) {

    if (!calprobe.locate(parser.byteval('V', 1))) return;

    calprobe.report();
    SERIAL_SMV(ECHO, "Calibration probe width X:", calprobe.width[X_AXIS], 3);
    SERIAL_MV(" Y:", calprobe.width[Y_AXIS], 3);
    SERIAL_MV(" range X:", calprobe.range[X_AXIS], 3);
    SERIAL_MV(" Y:", calprobe.range[Y_AXIS], 3);
    SERIAL_EMV(" Z:", calprobe.range[Z_AXIS], 3);
  }

#endif // HAS_CALIBRATION_PROBE

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
 * calprobe.h - XYZ nozzle offset calibration probe
 *
 * A ball or a pin on a micro switch (Mellow MultiHead Zero, Nudge, Sexball probe...)
 * that triggers when a nozzle pushes it from any side or from above.
 * The probe is mounted on the bed and connected to Z_PROBE_PIN.
 *
 * The probe center is found by touching the probe from above and from four sides.
 * The center on each axis is the middle between two opposite contacts, so the probe
 * and nozzle sizes, the switch pre-travel and the axis backlash cancel out.
 */

#if HAS_CALIBRATION_PROBE

class CalibrationProbe {

  public: /** Constructor */

    CalibrationProbe() {}

  public: /** Public Parameters */

    enum ProbeResult : uint8_t { PROBE_CONTACT, PROBE_NO_CONTACT, PROBE_FAILED };

    // Settings, set to the configuration defaults at startup and changed by M1020
    static float    spread,         // Side touches start this far from the probe center
                    lower_z,        // Height of the side touches below the probe top
                    lift_z,         // Travel height above the probe top
                    tolerance,      // Max difference between the touches of one side
                    feedrate_slow;  // Speed of the precise touches (mm/s)
    static uint8_t  samples;        // Touches per side in the precise pass

    // Result of the last locate
    static bool     located;        // center[] holds the probe position
    static uint8_t  located_tool;   // Tool used to locate the probe
    static float    center[XYZ],    // X, Y of the probe center and Z of the probe top (native coordinates)
                    width[2],       // X, Y distance between opposite contacts (probe size + nozzle tip size)
                    range[XYZ];     // Max difference between the touches of one side

  public: /** Public Function */

    static bool is_triggered();

    /**
     * Move the active nozzle along one axis until the probe triggers.
     * The current position is updated to where the axis stopped.
     */
    static ProbeResult probe_move(const AxisEnum axis, const float distance, const float fr_mm_s);

    /**
     * Move back along the axis after a touch until the probe is released
     */
    static bool release(const AxisEnum axis, const int8_t dir);

    /**
     * Locate the probe with the active nozzle, starting above the known probe
     * position or, if the probe is not located yet, from the current position.
     * The result is stored in center[], width[] and range[].
     *
     *  verbose 0 = no output, 1 = side results, 2 = every touch
     */
    static bool locate(const uint8_t verbose);

    static void report();

  private: /** Private Function */

    static bool do_locate(const uint8_t verbose);
    static ProbeResult guarded_move(const float target[XYZ], const float fr_mm_s);
    static bool travel(const float x, const float y, const float z, const float fr_mm_s);
    static bool reachable(const float target[XYZ]);
    static bool move_z(const float z);
    static bool move_xy(const float x, const float y);
    static bool touch(const AxisEnum axis, const int8_t dir, const float max_dist, const uint8_t count, float &contact, float &spread_mm, const uint8_t verbose);
    static bool probe_sides(float c[XYZ], const uint8_t count, const uint8_t verbose);
    static void report_touch(const AxisEnum axis, const int8_t dir, const float pos);

};

extern CalibrationProbe calprobe;

#endif // HAS_CALIBRATION_PROBE

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
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#if FAN_COUNT > 0

  #define CODE_M106
  #define CODE_M107

  /**
   * M106: Set Fan Speed
   *
   *  P<index>  Fan index, if more than one fan
   *  S<int>    Speed between 0-255
   *  F<int>    Set PWM frequency
   *  H<int>    Set Auto mode - H=7 for controller - H-1 for disabled
   *  U<int>    Fan Pin
   *  L<int>    Min Speed
   *  I<bool>   Inverted pin output
   */
  inline void gcode_M106(void) {

    if (printer.debugSimulation()) return;

    const uint8_t speed = parser.byteval('S', 255),
                  f     = parser.byteval('P');

    if (f < FAN_COUNT) {

      bool config = false;
      Fan *fan = &fans[f];

      if (parser.seen('U')) {
        // Put off the fan
    	config = true;
    	pin_t new_pin = parser.value_pin();
    	if (new_pin!= fan->pin)
    	{
            fan->Speed = 0;
            fan->pin = new_pin;
            SERIAL_LM(ECHO, MSG_CHANGE_PIN);
    	}
      }

      if (parser.seen('I'))
      {
        fan->setHWInverted(parser.value_bool());
        config = true;
      }

      if (parser.seen('H'))
      {
        fan->SetAutoMonitored(parser.value_int());
        config = true;
      }

      fan->min_Speed  = parser.byteval('L', fan->min_Speed);
      fan->freq       = parser.ushortval('F', fan->freq);

      if (!config)
      {
		  #if ENABLED(FAN_KICKSTART_TIME)
			if (fan->Kickstart == 0 && speed > fan->Speed && speed < 85) {
			  if (fan->Speed) fan->Kickstart = FAN_KICKSTART_TIME / 100;
			  else            fan->Kickstart = FAN_KICKSTART_TIME / 25;
			}
		  #endif


		  fan->Speed = fan->min_Speed + (speed * (255 - fan->min_Speed)) / 255;
      }

      if (!parser.seen('S')) {
        char response[70];
        sprintf_P(response, PSTR("Fan: %i pin: %i frequency: %uHz min: %i inverted: %s"),
            (int)f,
            (int)fan->pin,
            (uint16_t)fan->freq,
            (int)fan->min_Speed,
            (fan->isHWInverted()) ? "true" : "false"
        );
        SERIAL_TXT(response);

        // Auto Fan
        if (fan->autoMonitored!=-1) SERIAL_MSG(" Autofan on:");
        LOOP_HOTEND() {
          if (fan->autoMonitored==h) SERIAL_MV(" H", (int)h);
        }
        if (fan->autoMonitored==7) SERIAL_MSG(" Controller");
        if (fan->autoMonitored==8) SERIAL_MSG(" All hotends");
        if (fan->autoMonitored==9) SERIAL_MSG(" Chamber");

        SERIAL_EOL();
      }
    }
  }

  /**
   * M107: Fan Off
   */
  inline void gcode_M107(void) {
    const uint8_t f = parser.byteval('P');
    if (f < FAN_COUNT) fans[f].Speed = 0;
  }

#endif // FAN_COUNT > 0

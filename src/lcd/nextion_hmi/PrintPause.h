/*
 * PrintPause.h
 *
 *  Created on: 28 ????. 2018 ?.
 *      Author: Azarov
 */

#ifndef SRC_LCD_NEXTION_HMI_PRINTPAUSE_H_
#define SRC_LCD_NEXTION_HMI_PRINTPAUSE_H_

#if ENABLED(NEXTION_HMI)


enum PrintPauseStatus {NotPaused, WaitingToPause, Pausing, Paused, Resuming };

namespace PrintPause {

  extern bool CanPauseNow;
  extern bool SdPrintingPaused;
  extern PrintPauseStatus Status;

  void DoPauseExtruderMove(AxisEnum axis, const float &length, const float fr);

  bool PausePrint(const float &retract);
  void ResumePrint(const float& purge_length=0);
  void RestoreTemperatures();


}


#endif





#endif /* SRC_LCD_NEXTION_HMI_PRINTPAUSE_H_ */

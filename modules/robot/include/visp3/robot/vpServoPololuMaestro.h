/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Common features for Pololu Maestro Servo Motor.
 */

#ifndef _vpServoPololuMaestro_h_
#define _vpServoPololuMaestro_h_

#include <visp3/core/vpConfig.h>

#include "RPMSerialInterface.h"
#include <iostream>
#include <string>

#ifdef VISP_HAVE_RAPA_POLOLU_MAESTRO

class VISP_EXPORT vpServoPololuMaestro
{
public:
  vpServoPololuMaestro();
  vpServoPololuMaestro(RPM::SerialInterface *interface, int channel);
  virtual ~vpServoPololuMaestro();

  bool checkConnection();
  int angle2PWM(float angle);
  float PWM2Angle(int PWM);
  int setPositionAngle(float targetAngle, unsigned short speed = 0);
  int setPositionPWM(int targetPWM, unsigned short speed = 0);
  int setSpeed(unsigned short speed);
  int setSpeedDegS(float speedRadS);
  int setVelocityCmd(short velocityCmdSpeed);
  unsigned short getPosition();
  float getPositionAngle();
  unsigned short getSpeed();
  void getRangeAngle(float *minAngle, float *maxAngle, float *rangeAngle);
  void getRangePWM(int *minPWM, int *maxPWM, int *rangePWM);
  void stopVelCmd();
  float speedToDeGS(short speed);
  short degSToSpeed(float speedDegS);

private:

  RPM::SerialInterface *m_interface;
  int m_channel;
  bool m_FlagVelCmdRunning;

  unsigned short m_position;
  unsigned short m_speed;
  int m_velocityDirection;

// ranges
  int m_minPWM = 2800;
  int m_maxPWM = 8800;
  int m_rangePWM = m_maxPWM - m_minPWM;
  float m_minAngle = -40;
  float m_maxAngle = 40;
  float m_rangeAngle = abs(m_minAngle) + abs(m_maxAngle);

  // private  methods
  void VelocityCmdThread();

};

#endif
#endif

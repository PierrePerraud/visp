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
 * Common features for Pololu Maestro PanTiltUnit.
 */

#ifndef _vpPTUPololuMaestro_h_
#define _vpPTUPololuMaestro_h_

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_RAPA_POLOLU_MAESTRO

#include <visp3/robot/vpServoPololuMaestro.h>

#include "RPMSerialInterface.h"
#include <iostream>
#include <string>
#include <thread>
#include <chrono>


using namespace std;

typedef enum { pan, tilt, yaw } Axe;

class VISP_EXPORT vpPTUPololuMaestro
{
public:
  vpPTUPololuMaestro();
  vpPTUPololuMaestro(int baudrate, std::string dev);
  virtual ~vpPTUPololuMaestro();
  int setConnection(std::string dev, int baudrate, std::string error_msg);
  int getPositionAngle(float *angle, Axe axe);
  int setPositionAngle(float angle, unsigned short speed = 0, Axe axe = pan);
  int setVelocityCmd(short speed, Axe axe = pan);
  int stopVelocityCmd(Axe axe);
  void getRange(float *minAngle, float *maxAngle, float *rangeAngle, Axe axe = pan);

private:

  // Serial connection parameters
  int m_baudrate;
  std::string m_error_msg;
  std::string m_dev;
  RPM::SerialInterface *m_serialInterface;


  vpServoPololuMaestro m_pan;
  vpServoPololuMaestro m_tilt;
};

#endif
#endif

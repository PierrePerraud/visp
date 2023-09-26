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

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_RAPA_POLOLU_MAESTRO

#include "RPMSerialInterface.h"
#include <visp3/robot/vpPTUPololuMaestro.h>

/*!
 * Default constructor.
 */

vpPTUPololuMaestro::vpPTUPololuMaestro()
{
  this->m_baudrate = 9600;
  this->m_dev = "/dev/ttyACM0";
  this->setConnection(this->m_dev, this->m_baudrate, this->m_error_msg);

  // m_pan = vpServoPololuMaestro(this->m_serialInterface, 0);
  // m_tilt = vpServoPololuMaestro(this->m_serialInterface, 1);
  // m_yaw = vpServoPololuMaestro(this->m_serialInterface, 2); // not used for now
}

/*!
 * Value constructor.
 *
 * \param baudrate : baudrate used for the serial communication
 *
 * \param dev : name of the serial interface used for communication
 *
 */

vpPTUPololuMaestro::vpPTUPololuMaestro(int baudrate, std::string dev)
{
  this->m_baudrate = baudrate;
  this->m_dev = dev;
  this->setConnection(this->m_dev, this->m_baudrate, this->m_error_msg);

  m_pan = vpServoPololuMaestro(this->m_serialInterface, 0);
  m_tilt = vpServoPololuMaestro(this->m_serialInterface, 1);
}

/*!
 * Destructor.
 */
vpPTUPololuMaestro::~vpPTUPololuMaestro() {}

/*!
 * initiate the serial connection with the Pololo board
 *
 * \param dev : name of the serial interface used for communication
 *
 * \param baudrate : baudrate used for the serial communication
 *
 * \param error_msg : error message for serial communication
 *
 * \return error : return 1 if the serial connection is not set
 *
 */
int vpPTUPololuMaestro::setConnection(std::string dev, int baudrate, std::string error_msg)
{
  this->m_serialInterface = RPM::SerialInterface::createSerialInterface(dev, baudrate, &error_msg);
  std::cout << error_msg;
  std::cout << "Serial ls /dev         Started!\n";
  std::cout << m_serialInterface->isOpen() << "\n";
  if (!m_serialInterface->isOpen()) {
    std::cout << "Serial Communication Failed!\n";
    return 1;
  } else {
    return 0;
  }
}

/*!
 *  set position in degree and the maximum speed of displacement for a given axe
 *
 * \param angle : angle to reach , in degree
 *
 * \param speed : maximum speed for movement in units of (0.25 μs)/(10 ms). You can use the
 * vpServoPololuMaestro::degSToSpeed method for conversion
 *
 * \param axe : one of the axe define in vpPTUPololuMaestro.h Axe enum
 *
 * \return error : return 1 if an error is encountered
 *
 */
int vpPTUPololuMaestro::setPositionAngle(float angle, unsigned short speed, Axe axe)
{
  switch (axe) {
  case pan:
    return m_pan.setPositionAngle(angle, speed);
    break;
  case tilt:
    return m_tilt.setPositionAngle(angle, speed);
    break;
  default:

    std::cout << "no corresponding axe" << std::endl;
    return 1;
    break;
  }
}

/*!
 *  get ranges for a given axe
 *
 * \param minAngle : pointer to minimum range of the servo
 *
 * \param maxAngle : pointer to maximum range of the servo
 *
 * \param rangeAngle : pointer to the range of the servo
 *
 * \param axe : one of the axe define in vpPTUPololuMaestro.h Axe enum
 *
 */
void vpPTUPololuMaestro::getRange(float *minAngle, float *maxAngle, float *rangeAngle, Axe axe)
{
  switch (axe) {
  case pan:
    m_pan.getRangeAngle(minAngle, maxAngle, rangeAngle);
    break;
  case tilt:
    m_tilt.getRangeAngle(minAngle, maxAngle, rangeAngle);
    break;
  default:

    std::cout << "no corresponding axe" << std::endl;
    *minAngle = 0;
    *maxAngle = 0;
    *rangeAngle = 0;
    break;
  }
}

/*!
 *  Set and start the velocity command for a given axe
 *
 * \param speed : speed for movement in units of (0.25 μs)/(10 ms). You can use the vpServoPololuMaestro::degSToSpeed
 * method for conversion
 *
 * \param axe : one of the axe define in vpPTUPololuMaestro.h Axe enum
 *
 * \return error : return 1 if an error is encountered
 *
 */
int vpPTUPololuMaestro::setVelocityCmd(short speed, Axe axe)
{
  std::cout << "start vel cmd" << std::endl;
  switch (axe) {
  case pan:
    return m_pan.setVelocityCmd(speed);
    break;
  case tilt:
    return m_tilt.setVelocityCmd(speed);
    break;
  default:
    std::cout << "no corresponding axe" << std::endl;
    return 1;
    break;
  }
}

/*!
 *  Stop the velocity command for a given axe
 *
 * \param axe : one of the axe define in vpPTUPololuMaestro.h Axe enum
 *
 * \return error : return 1 if an error is encountered
 *
 */
int vpPTUPololuMaestro::stopVelocityCmd(Axe axe)
{
  switch (axe) {
  case pan:
    m_pan.stopVelCmd();
    break;
  case tilt:
    m_tilt.stopVelCmd();
    break;
  default:
    std::cout << "no corresponding axe" << std::endl;
    return 1;
    break;
  }
  return 0;
}

/*!
 *  get the current position of a servo motor in degree
 *
 * \param axe : one of the axe define in vpPTUPololuMaestro.h Axe enum
 *
 * \return value : return 1 if an error is encountered
 *
 */
int vpPTUPololuMaestro::getPositionAngle(float *angle, Axe axe)
{
  switch (axe) {
  case pan:
    *angle = m_pan.getPositionAngle();
    break;
  case tilt:
    *angle = m_tilt.getPositionAngle();
    break;
  default:
    std::cout << "no corresponding axe" << std::endl;
    return 1;
    break;
  }
  return 0;
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_robot.a(vpPTUPololuMaestro.cpp.o) has no symbols
void dummy_vpPTUPololuMaestro(){};
#endif

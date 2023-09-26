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

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_RAPA_POLOLU_MAESTRO

#include "RPMSerialInterface.h"
#include <visp3/robot/vpServoPololuMaestro.h>

#include <chrono>
#include <thread>


std::chrono::milliseconds millis(1);

/*!
 * Default constructor.
 */
vpServoPololuMaestro::vpServoPololuMaestro()
{
  // std::string error_msg;
  // this->m_interface = RPM::SerialInterface::createSerialInterface("/dev/ttyACM0", 9600, &error_msg);
  // this->m_channel = 0;
  // this->m_FlagVelCmdRunning = false;
  // std::thread t(&vpServoPololuMaestro::VelocityCmdThread, this);
  // t.detach();


  // std::cout << " default servo created on channel : "<< this->m_channel << std::endl;
}

/*!
 * Value constructor
 *
 * \param interface : serial interface for communication.
 *
 * \param channel : channel number in which the servo is connected on the pololu board
 *
 */
vpServoPololuMaestro::vpServoPololuMaestro(RPM::SerialInterface *interface, int channel)
{
  this->m_interface = interface;
  this->m_channel = channel;
  this->m_FlagVelCmdRunning = false;
  std::thread t(&vpServoPololuMaestro::VelocityCmdThread, this);
  t.detach();

  std::cout << " value servo created on channel : "<< this->m_channel << std::endl;
}

/*!
 * Destructor.
 */
vpServoPololuMaestro::~vpServoPololuMaestro() { }

/*!
 * convert angles to PWM for servo commands
 *
 * \param angle : angle, in degree to be converted
 *
 * \return PWM : corresponding PWM for the angle
 */
int vpServoPololuMaestro::angle2PWM(float angle)
{
  return ((angle + abs(m_minAngle)) / m_rangeAngle) * m_rangePWM + m_minPWM;
}

float vpServoPololuMaestro::PWM2Angle(int PWM) { return (PWM * (m_rangeAngle / m_rangePWM) + m_minAngle); }

/*!
 * return position in PWM
 *
 * \return position : return current position in PWM
 */
unsigned short vpServoPololuMaestro::getPosition()
{
  m_interface->getPositionCP(this->m_channel, this->m_position);

  std::cout << "current Position : " << this->m_position << std::endl;
  return this->m_position;
}

/*!
 * return position in PWM
 *
 * \return positionAngle : return current position in deg
 */
float vpServoPololuMaestro::getPositionAngle()
{
  m_interface->getPositionCP(this->m_channel, this->m_position);
  float positionAngle = this->PWM2Angle(this->m_position);
  std::cout << "current angle : " << positionAngle << std::endl;
  return positionAngle;
}

/*!
 * set the position to reach in PWM
 *
 * \param targetPWM : position in PWM to reach
 *
 * \param speed : OPTIONAL : speed to use for movement in units of (0.25 μs)/(10 ms). Default is 0, maximum speed
 *
 * \return error : return 1 if an error is encountered
 */
int vpServoPololuMaestro::setPositionPWM(int targetPWM, unsigned short speed)
{
  if ((this->m_minPWM <= targetPWM) && (targetPWM <= this->m_maxPWM)) {
    this->setSpeed(speed);
    // std::cout << "position (PWM):"<<targetPWM<<" desired speed :"<<speed<<" \n";
    // std::cout << "current speed :"<<getSpeed()<<" \n";
    m_interface->setTargetCP(this->m_channel, targetPWM);
    return 0;
  }
  else {
    std::cout << "given position : " << targetPWM
      << " is outside of the servo range. You can check the range using the method getRangePWM()" << std::endl;
    return 1;
  }
}

/*!
 * Get min, max and range for PWM cmd
 *
 * \param minPWM : address of the min range value for PWM control
 *
 * \param maxPWM : address of the max range value for PWM control
 *
 * \param rangePWM : address of the range value for PWM control
 *
 */
void vpServoPololuMaestro::getRangePWM(int *minPWM, int *maxPWM, int *rangePWM)
{
  *minPWM = this->m_minPWM;
  *maxPWM = this->m_maxPWM;
  *rangePWM = this->m_rangePWM;
}

/*!
 * Get min, max and range for angle cmd
 *
 * \param minAngle : address of the min range value for angle control
 *
 * \param maxAngle : address of the max range value for angle control
 *
 * \param rangeAngle : address of the range value for angle control
 *
 */
void vpServoPololuMaestro::getRangeAngle(float *minAngle, float *maxAngle, float *rangeAngle)
{
  *minAngle = this->m_minAngle;
  *maxAngle = this->m_maxAngle;
  *rangeAngle = this->m_rangeAngle;
}

/*!
 * set the position to reach in angle
 *
 * \param targetAngle : position in angle to reach
 *
 * \param speed : OPTIONAL : speed to use for movement in units of (0.25 μs)/(10 ms). Default is '0' and will use
 * maximum speed
 *
 * \return error : return 1 if an error is encountered
 *
 */
int vpServoPololuMaestro::setPositionAngle(float targetAngle, unsigned short speed)
{
  if ((this->m_minAngle <= targetAngle) && (targetAngle <= this->m_maxAngle)) {
    int targetPWM = angle2PWM(targetAngle);
    setPositionPWM(targetPWM, speed);
    return 0;
  }
  else {
    std::cout << "given position : " << targetAngle
      << " is outside of the servo range. You can check the range using the method getRangeAngle()"
      << std::endl;
    return 1;
  }
}

/*!
 *  set the speed of the motor movements
 *
 * \param speed : speed to use for movement in units of (0.25 μs)/(10 ms). no speed (0) will use maximum speed
 *
 * \return error : return 1 if an error is encountered
 */
int vpServoPololuMaestro::setSpeed(unsigned short speed)
{
  if (speed <= 1000) {
    m_interface->setSpeedCP(this->m_channel, speed);
    this->m_speed = speed;
    return 0;
  }
  else {
    std::cout << "given speed : " << speed << " is outside of the servo speed range. range is from 0 to 1000"
      << std::endl;
    return 1;
  }
}

/*!
 *  return the current speed parameter
 *
 * \return speed : speed to use for movement in units of (0.25 μs)/(10 ms). no speed (0) will use maximum speed
 *
 */
unsigned short vpServoPololuMaestro::getSpeed() { return this->m_speed; }

/*!
 *  set the speed of the motor movements
 *
 * \param speed : speed to use for movement in units of (0.25 μs)/(10 ms). no speed (0) will use maximum speed
 *
 * \return error : return 1 if an error is encountered
 *
 */
int vpServoPololuMaestro::setSpeedDegS(float speedDegS)
{
  unsigned short speed =
    (unsigned short)abs(this->degSToSpeed(speedDegS)); // making sure the speed is positive and convert it tu ushort

  return this->setSpeed(speed);
}

/*!
 *  method to convert Pololu's speed to deg/s speed
 *
 * \param speed : speed in units of (0.25 μs)/(10 ms).
 *
 * \return speedDegS : speed converted to deg/s
 *
 */
float vpServoPololuMaestro::speedToDeGS(short speed) { return (speed * 100) * (this->m_rangeAngle / this->m_rangePWM); }

/*!
 *  method to convert deg/s speed into Pololu's speed
 *
 * \param speedDegS : speed converted to deg/s
 *
 * \return speed : speed in units of (0.25 μs)/(10 ms).
 *
 */
short vpServoPololuMaestro::degSToSpeed(float speedDegS)
{
  return (speedDegS / 100) * (this->m_rangePWM / this->m_rangeAngle);
}

/*!
 *  set the speed of the motor movements and start the velocity command thread. The motor will move to the edge of the
 * range at the given speed.
 *
 * \param speed : speed to use for movement in units of (0.25 μs)/(10 ms). no speed (0) will use maximum speed
 *
 * \return error : return 1 if an error is encountered
 */
int vpServoPololuMaestro::setVelocityCmd(short velocityCmdSpeed)
{
  if (velocityCmdSpeed <= 1000) {
    this->m_speed = abs(velocityCmdSpeed);
    if (velocityCmdSpeed >= 0) {
      this->m_velocityDirection = m_maxPWM;
    }
    else {
      this->m_velocityDirection = m_minPWM;
    }
    this->m_FlagVelCmdRunning = true;

    return 0;
  }
  else {
    std::cout << "given velocityCmdSpeed : " << velocityCmdSpeed
      << " is outside of the servo speed range. range is from 0 to 1000" << std::endl;
    return 1;
  }
}


/*!
 *  Thread use for Velocity control. This thread is launch in the constructor of the object and, unless crashes, will
 * run until the process is ended. If the m_FlagVelCmdRunning is set to TRUE, by invoking the setVelocityCmd method, the
 * motor will go to the edge of the motor range using the speed set in setVelocityCmd. The velocity command can be
 * stopped invoking the stopVelCmd() method.
 *
 */
void vpServoPololuMaestro::VelocityCmdThread()
{
  int antispamCounter = 0;
  while (true) {
    if (this->m_FlagVelCmdRunning) {
      this->getPosition();
      if (int(this->m_position) != (this->m_velocityDirection)) {
        this->setPositionPWM(this->m_velocityDirection, this->m_speed);
        std::cout << "vel cmd running, m_velocityDirection is : " << this->m_velocityDirection
          << " and currently at : " << this->m_position << std::endl;
      }
      else {
        std::cout << "edge of range reach" << std::endl;
      }
    }
    else {
      if (antispamCounter == 100) {
        std::cout << "waiting"
          << "flag is : " << this->m_FlagVelCmdRunning << std::endl;
        antispamCounter = 0;
      }
      else {
        antispamCounter++;
      }
    }
    std::this_thread::sleep_for(10 * millis);
  }

  std::cout << " Vel cmd stopped unexpectedly  \n";

  return;
}
/*!
 *  Stop the velocity command thread.
 *
 */
void vpServoPololuMaestro::stopVelCmd()
{
  std::cout << " try to stop vel cmd \n";
  this->m_FlagVelCmdRunning = false;

  std::this_thread::sleep_for(10 * millis);
  this->setPositionPWM(this->getPosition());
}


#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_robot.a(vpServoPololuMaestro.cpp.o) has no symbols
void dummy_vpServoPololuMaestro() { };
#endif

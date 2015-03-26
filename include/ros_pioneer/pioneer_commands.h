/*********************************************************************
* pioneer_commands.h
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, University of Patras
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of University of Patras nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Authors: Aris Synodinos
*********************************************************************/

#ifndef PIONEER_COMMANDS_H
#define PIONEER_COMMANDS_H

namespace ros_pioneer {

typedef enum
{
  SYNC0     = 0,        // First syncronization packet
  SYNC1     = 1,        // Second syncronization packet
  SYNC2     = 2         // Third syncronization packet
}Init_t;

typedef enum
{
  PULSE       = 0,      // Reset server watchdog.
  OPEN        = 1,      // Start up servers.
  CLOSE       = 2,      // Close servers and client connection.
  SETO        = 7,      // Reset local position to 0,0,0 origin.
  CONFIG      = 18,     // Request a configuration SIP.
  STOP        = 29,     // Stop the robot; motors remain enabled
  E_STOP      = 55,     // Emergency stop; very abrupt by overriding deceleration.
  STEP        = 64,     // Single-step mode (simulator only)
  ARM_INFO    = 70,     // Request an ARMINFOpac server-information packet
  ARM_INIT    = 72,     // Warm-reset the Pioneer Arm
  ARM_CHECK   = 73,     // Has Arm servers check to make sure it’s still
                        // responding to commands; reflects in status byte of
                        // ARMpac.
  ARM_PARK    = 76,     // Sends all joints to their HOME positions and shuts
                        // power off.
  SKRMAINT    = 117,    // Engage Seekur microcontroller maintenance mode
                        // (factory use only).
  SYSKILL_RECENTER = 119, // Recenter the wheels, then turns off the robot. (Seekur only)
  RECENTERWHEELS = 120, // Recenter the wheels. (Seekur only)
  RESET       = 253,    // Force a power on-like reset of the microcontroller.
  MAINTENANCE = 255     // Engage microcontroller maintenance (ARSHstub) mode.
}Void_t;

typedef enum
{
  DIGOUT      = 30,     // Set (1) or reset (0) User Output ports. Bits 8-15 is
                        // a byte mask that selects, if set (1), the output
                        // port(s) for change; Bits 0-7 set (1) or reset (0)
                        // the selected port(s).
  VEL2        = 32,     // Set independent wheel velocities; bits 0-7 for right
                        // wheel, bits 8-15 for left wheel; in 20mm/sec
                        // increments.
  PTUPOS      = 41,     // Set pulse-width for position servo control; lsb
                        // is pulse width in 100μs units; msb addresses
                        // mux'd RC0-4 on Experimenter's Module.
  ARM_POS     = 77,     // Sends joint (1st byte) to the position specified
                        // in the 2nd byte, subject to end-limits specified
                        // in FLASH parameters
  ARM_SPEED   = 78,     // Delay in milliseconds (2nd byte) between
                        // incremental steps to control the speed and motion
                        // of the joint (1st byte)
  SEEKURPDB   = 116     // Turn a Seekur PDB power port on or off. First byte
                        // indicates which port, second byte indicates state
                        // (1=on, 0=off). See Power Supply for locations of
                        // power output connectors and typical devices
                        // attached to which ports.. Seekur and Seekur Jr. only.
}TwoBytes_t;

typedef enum
{
  ENABLE      = 4,      // 1=enable; 0=disable the motors.
  SETA        = 5,      // Set translation acceleration, if positive,
                        // or deceleration, if negative; in mm/sec2.
  SETV        = 6,      // Set maximum/move translation velocity; mm/sec.
  MOVE        = 8,      // Translate (+) forward or (-) back mm distance
                        // at SETV speed
  ROTATE      = 9,      // Rotate (+) counter- or (-) clockwise degrees/sec
                        // at SETRV limited speed.
  SETRV       = 10,     // Sets maximum/turn rotation velocity; degrees/sec.
  VEL         = 11,     // Translate at mm/sec forward (+) or backward (-)
                        // (SETV limited speed).
  HEAD        = 12,     // Turn at SETRV speed to absolute heading;
                        // ±degrees (+= counterclockwise).
  DHEAD       = 13,     // Turn at SETRV speed relative to current heading;
                        // (+) counter- or (–) clockwise degrees.
  JOYREQUEST  = 17,     // Request one or continuous stream (>1) or stop (0)
                        // joystick SIPs
  ENCODER     = 19,     // Request one, a continuous stream (>1), or stop (0)
                        // encoder SIPs.
  RVEL        = 21,     // Rotate robot at (+) counter- or (-) clockwise;
                        // degrees/sec (SETRV limit).
  DCHEAD      = 22,     // Adjust heading relative to last setpoint;
                        // ± degrees (+= ccw)
  SETRA       = 23,     // Change rotation de(-) or (+)acceleration, in
                        // degrees/sec2
  IMUREQUEST  = 26,     // Use argument value 1 to request one IMU packet.
                        // Use argument value 2 to request continuous IMU
                        // packets every cycle. Send argument value 0 to
                        // stop. See Inertial Measurement Unit (IMU) on
                        // page 53 for details.
  SONAR       = 28,     // 1=enable, 0=disable all the SONAR; otherwise, use
                        // bits 1-3 to specify an individual array number 1-4.
  TIMER       = 31,     // Initiate user input timer, triggering an event
                        // with specified PIN
  GRIPPER     = 33,     // Gripper server commands. See the Gripper or PeopleBot
                        // Manual for details.
  KICK        = 34,     // Trigger digital output port OD0 for ms milliseconds
                        // (one-shot).
  ADSEL       = 35,     // Selects the A/D port value in standard SIP.
  GRIPPERVAL  = 36,     // Gripper server values. See the Gripper or PeopleBot
                        // Manual for details.
  GRIPREQUEST = 37,     // Request one, a continuous stream (>1), or stop (0)
                        // Gripper SIPs.
  GYROCALCW   = 38,     // Set the clockwise rotation calibration value for the
                        // gyro accessory.
  GYROCALCCW  = 39,     // Set the counterclockwise rotation calibration value
                        // for the gyro accessory.
  IOREQUEST   = 40,     // Request one (1), a continuous stream (>1), or stop
                        // (0) IO SIPs.
  GETAUX      = 43,     // Request to retrieve 1-200 bytes from the AUX1 serial
                        // port; 0 flushes the buffer.
  BUMPSTALL   = 44,     // Stall robot if no (0), only front (1) while moving
                        // forward, only rear (2) while moving backward, or
                        // either (3) bumpers contacted when robot moving in
                        // related direction.
  TCM2        = 45,     // TCM2 module commands; 0=module off, no readings;
                        // 1=compass only, readings in standard SIP; 2=send one
                        // TCM2 packet; 3=send continuous (each cycle) TCM2
                        // packets; 4=user calibration; 5=auto calibration;
                        // 6=stop auto-calibration, send one packet, revert to
                        // mode 1; 7=soft reset
  JOYDRIVE    = 47,     // 1=allow joystick drive from port while connected with
                        // a client; 0 (default) disallows.
  SONARCYCLE  = 48,     // Change the SONAR cycle time; in milliseconds.
  HOSTBAUD    = 50,     // Change the HOST serial port baud rate to 0=9600,
                        // 1=19200, 2=38400, 3=57600, or 4=115200.
  AUX1BAUD    = 51,     // Change the AUX1 serial port baud rate (see HOSTBAUD).
  AUX2BAUD    = 52,     // Change the AUX2 serial port baud rate (see HOSTBAUD).
  AUX3BAUD    = 53,     // Change the AUX3 serial port baud rate (see HOSTBAUD).
  M_STALL     = 56,     // Argument 1=MOTORS button off causes a stall
  GYROREQUEST = 58,     // If client-side (HasGyro 1), request one, a continuous
                        // stream (>1), or stop (0) Gyro SIPs. If server-side
                        // (HasGyro 2), 0 disable or 1 to enable the gyro.
  GETAUX3     = 61,     // Request to retrieve 1-200 bytes from the device
                        // connected at the AUX3 serial port; 0 flushes the
                        // buffer.
  GETAUX2     = 67,     // Request to retrieve 1-200 bytes from the device
                        // connected at the AUX2 serial port; 0 flushes the
                        // buffer.
  CHARGE      = 68,     // 0=release; 1=deploy autocharge-docking mechanism.
  ARM_STATUS  = 71,     // Request one (1), a continuous stream (>1), or stop
                        // (0) ARMpac SIPs.
  ARM_POWER   = 74,     // Switches power off (0) or on (1) to all the Arm
                        // servos
  ARM_HOME    = 75,     // Sends all (>6) or specified joint (1-6) to the
                        // HOME position
  ARM_STOP    = 79,     // Stop the specified (1-6) or all (255) moving
                        // joints
  ARM_AUTOPARK= 80,     // Disable the autopark watchdog (0) or reset it to
                        // some time in seconds other than default
                        // AutoParkTimer in FLASH
  ARM_GRIPPARK= 81,     // Disable the gripper watchdog (0) or reset it to
                        // some time in seconds other than the default
                        // GripperParkTimer
  ROTKP       = 82,     // Change working rotation Proportional PID value.
  ROTKV       = 83,     // Change working rotation Derivative PID value.
  ROTKI       = 84,     // Change working rotation Integral PID value.
  TRANSKP     = 85,     // Change working translation Proportional PID value.
  TRANSKV     = 86,     // Change working translation Derivative PID value.
  TRANSKI     = 87,     // Change working translation Integral PID value.
  REVCOUNT    = 88,     // Change working differential encoder count.
  DRIFTFACTOR = 89,     // Change working drift factor.
  SOUNDTOG    = 92,     // 0=mute User Control piezo; 1 = enable.
  TICKSMM     = 93,     // Change working encoder ticks per millimeter
                        // tire travel.
  LATVEL      = 110,    //(Seekur only) Set velocity for sideways translation
                        // (mm/sec). Positive argument values translate to the
                        // right, negative values translate to the left. May be
                        // combined with VEL and RVEL for simultaneous motion.
                        // Maximum speed is limited with SETV and stored firmware
                        // parameter. Only available on Seekur.
  LATACCEL    = 113,    // If positive, set acceleration used for LATVEL commands
                        // in mm/sec2. If negative, set deceleration used for
                        // LATVEL commands in mm/sec2.
  BATTEST     = 250,    // Artificially set the battery voltage; argument in
                        // tens volts (100=10V); 0 to revert to real voltage
  DIGTEMPTEST = 251,    // Artificially set the digital temperature; argument
                        // in degrees C; 0 to revert to real temperature.
  ANTEMPTEST  = 252     // Artificially set the analog temperature; argument
                        // in degrees C; 0 to revert to real temperature.
}Int_t;

typedef enum
{
  POLLING     = 3,      // Change SONAR polling sequence.
  SAY         = 15,     // Play up to 20 duration, tone sound pairs through
                        // User Control Panel piezo speaker.
  TTY2        = 42,     // Sends string argument to serial device connected to
                        // AUX1 serial port.
  LCDWRITE    = 59,     // Display a message on the LCD accessory: byte
                        // 0=starting column (1-19); byte 1=starting row (1-4);
                        // byte 2=1 if clear line contents first, otherwise 0;
                        // bytes 3- = up to 20 chars, NULL-terminated.
  TTY4        = 60,     // Send string argument out to device connected at AUX3
                        // serial port.
  TTY3        = 66      // Send string argument out to device connected at AUX2
                        // serial port.
}Str_t;

typedef enum{
  ARGINT      = 59,     // Positive Integer
  ARGNINT     = 27,     // Negative or Unsigned Integer
  ARGSTR      = 43      // String Argument
}Types_t;

typedef enum{
  STANDARDmpac= 48,
  STANDARDMpac= 63,
  CONFIGpac   = 32,
  SERAUXpac   = 176,
  SERAUX2pac  = 184,
  SERAUX3pac  = 200,
  ENCODERpac  = 144,
  TCM2pac     = 192,
  IOpac       = 240,
  IMUpac      = 154,
  JOYSTICKpac = 248,
  GRIPPERpac  = 224,
  GYROpac     = 152,
  ARMINFOpac  = 161,
  ARMpac      = 160
}Sip_t;

}

#endif // PIONEER_COMMANDS_H

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <units/angle.h>
#include <units/time.h>
#include <units/voltage.h>

#include <frc/trajectory/TrapezoidProfile.h>
#include <rev/CANSparkMax.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/velocity.h>

#include <numbers>

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

// test wheel angle feedback options
//#define USE_ROBORIO_ANALOG_INPUT            // Absolute Angle Sensor on RoboRIO analog input
#define USE_SPARK_MAX_ANALOG_INPUT          // Absolute Angle Sensor on Spark MAX analog input
//#define USE_SPARK_MAX_ABS_ENCODER_INPUT     // Through-hole Encoder on Spark MAX frequency-pwm input


namespace DriveConstants {

// CAN Sparkmax id numbers
constexpr int kFrontLeftDriveMotorPort = 10;
constexpr int kFrontRightDriveMotorPort = 12;
constexpr int kBackLeftDriveMotorPort = 14;
constexpr int kBackRightDriveMotorPort = 16;

constexpr int kFrontLeftTurningMotorPort = 11;
constexpr int kFrontRightTurningMotorPort = 13;
constexpr int kBackLeftTurningMotorPort = 15;
constexpr int kBackRightTurningMotorPort = 17;

// Anolog input ports on roborio
constexpr int kFrontLeftTurningEncoderPort = 0;
constexpr int kFrontRightTurningEncoderPort = 1;
constexpr int kBackLeftTurningEncoderPort = 2;
constexpr int kBackRightTurningEncoderPort = 3;

// Offsets in radians for the encoders. the first number to to make zero forward, after that we
// subtract an additional pi to make the full range -pi to pi instead of 0 to 2pi
#ifdef USE_ROBORIO_ANALOG_INPUT            // Absolute Angle Sensor on RoboRIO analog input
constexpr double kFrontLeftDriveEncoderOffset = 1.113 + std::numbers::pi;
constexpr double kFrontRightDriveEncoderOffset = 2.787 + std::numbers::pi;
constexpr double kBackLeftDriveEncoderOffset =  -0.155 + std::numbers::pi;
constexpr double kBackRightDriveEncoderOffset = -1.850 + std::numbers::pi;
#endif
#ifdef USE_SPARK_MAX_ANALOG_INPUT          // Absolute Angle Sensor on Spark MAX analog input
constexpr double kFrontLeftDriveEncoderOffset = 0.0;
constexpr double kFrontRightDriveEncoderOffset = 2.787 + std::numbers::pi;
constexpr double kBackLeftDriveEncoderOffset =  -0.155 + std::numbers::pi;
constexpr double kBackRightDriveEncoderOffset = -1.850 + std::numbers::pi;
#endif
#ifdef USE_SPARK_MAX_ABS_ENCODER_INPUT     // Through-hole Encoder on Spark MAX frequency-pwm input
constexpr double kFrontLeftDriveEncoderOffset = 0.0;
constexpr double kFrontRightDriveEncoderOffset = 2.787 + std::numbers::pi;
constexpr double kBackLeftDriveEncoderOffset =  -0.155 + std::numbers::pi;
constexpr double kBackRightDriveEncoderOffset = -1.850 + std::numbers::pi;
#endif

constexpr auto kRobotMaxLinearVelocity = 4.6_mps; // 4.6
constexpr auto kRobotMaxAngularVelocity = std::numbers::pi * 5_rad_per_s;

constexpr auto kDriveBaseRadius = 0.46_m;

}  // namespace DriveConstants

namespace ModuleConstants {
#ifdef USE_ROBORIO_ANALOG_INPUT            // Absolute Angle Sensor on RoboRIO analog input
// This is something to try and get rid of
// If feels like there should be something like this built into the systems
constexpr double ANALOG_TO_RAD_FACTOR = 1.2566;     // 0 to 5.0 volt = 2PI rad
constexpr double SPARK_MAX_ANALOG_TO_RAD_FACTOR = 1.9040;     // 0 to 3.3 volt = 2PI rad  should come back and revisit this to maybe fix anolog things.

constexpr double kWheelRadiusMeters = 0.0508;
constexpr int kEncoderResolution = 42;
constexpr double kGearRatio = 6.75;

constexpr double kDriveVelocityEncoderConversionFactor = (2.0 * std::numbers::pi * kWheelRadiusMeters 
                                                / (kGearRatio * kEncoderResolution));

constexpr double kDrivePositionEncoderConversionFactor = (2.0 * std::numbers::pi * kWheelRadiusMeters 
                                                / (kGearRatio));

constexpr auto kModuleMaxAngularVelocity =  std::numbers::pi * 9_rad_per_s;  // radians per second ?????????
constexpr auto kModuleMaxAngularAcceleration = std::numbers::pi * 20_rad_per_s / 1_s;  // radians per second^2
constexpr auto kModuleMaxLinearVelocity = 4.6_mps;

constexpr double kPModuleTurningController = 1;

constexpr double kPModuleDriveController = 1;
constexpr double kIModuleDriveController = 0.1;
constexpr double kDModuleDriveController = 0.1;
#endif

constexpr auto kModuleMaxLinearVelocity = 4.6_mps;

#ifdef USE_SPARK_MAX_ANALOG_INPUT          // Absolute Angle Sensor on Spark MAX analog input
// Invert the turning encoder, since the output shaft rotates in the opposite
// direction of the steering motor in the MAXSwerve Module.
constexpr bool kTurningEncoderInverted = true;

// The MAXSwerve module can be configured with one of three pinion gears: 12T,
// 13T, or 14T. This changes the drive speed of the module (a pinion gear with
// more teeth will result in a robot that drives faster).
constexpr int kDrivingMotorPinionTeeth = 14;

// Calculations required for driving motor conversion factors and feed forward
constexpr double kDrivingMotorFreeSpeedRps =
    5676.0 / 60;  // NEO free speed is 5676 RPM
constexpr units::meter_t kWheelDiameter = 0.0762_m;
constexpr units::meter_t kWheelCircumference =
    kWheelDiameter * std::numbers::pi;
// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
// teeth on the bevel pinion
constexpr double kDrivingMotorReduction =
    (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
constexpr double kDriveWheelFreeSpeedRps =
    (kDrivingMotorFreeSpeedRps * kWheelCircumference.value()) /
    kDrivingMotorReduction;

constexpr double kDrivingEncoderPositionFactor =
    (kWheelDiameter.value() * std::numbers::pi) /
    kDrivingMotorReduction;  // meters
constexpr double kDrivingEncoderVelocityFactor =
    ((kWheelDiameter.value() * std::numbers::pi) / kDrivingMotorReduction) /
    60.0;  // meters per second

constexpr double kTurningEncoderPositionFactor = (2 * std::numbers::pi) / 3.3;  // radians
constexpr double kTurningEncoderVelocityFactor = ((2 * std::numbers::pi) / 3.3) / 60.0;  // radians per second

constexpr units::radian_t kTurningEncoderPositionPIDMinInput = 0_rad;
constexpr units::radian_t kTurningEncoderPositionPIDMaxInput =
    units::radian_t{kTurningEncoderPositionFactor};

constexpr double kDrivingP = 0.04;
constexpr double kDrivingI = 0;
constexpr double kDrivingD = 0;
constexpr double kDrivingFF = (1 / kDriveWheelFreeSpeedRps);
constexpr double kDrivingMinOutput = -1;
constexpr double kDrivingMaxOutput = 1;

constexpr double kTurningP = 1;
constexpr double kTurningI = 0;
constexpr double kTurningD = 0;
constexpr double kTurningFF = 0;
constexpr double kTurningMinOutput = -1;
constexpr double kTurningMaxOutput = 1;

constexpr rev::CANSparkMax::IdleMode kDrivingMotorIdleMode =
    rev::CANSparkMax::IdleMode::kBrake;
constexpr rev::CANSparkMax::IdleMode kTurningMotorIdleMode =
    rev::CANSparkMax::IdleMode::kBrake;

constexpr units::ampere_t kDrivingMotorCurrentLimit = 50_A;
constexpr units::ampere_t kTurningMotorCurrentLimit = 20_A;
#endif

#ifdef USE_SPARK_MAX_ABS_ENCODER_INPUT     // Through-hole Encoder on Spark MAX frequency-pwm input
// Invert the turning encoder, since the output shaft rotates in the opposite
// direction of the steering motor in the MAXSwerve Module.
constexpr bool kTurningEncoderInverted = true;

// The MAXSwerve module can be configured with one of three pinion gears: 12T,
// 13T, or 14T. This changes the drive speed of the module (a pinion gear with
// more teeth will result in a robot that drives faster).
constexpr int kDrivingMotorPinionTeeth = 14;

// Calculations required for driving motor conversion factors and feed forward
constexpr double kDrivingMotorFreeSpeedRps =
    5676.0 / 60;  // NEO free speed is 5676 RPM
constexpr units::meter_t kWheelDiameter = 0.0762_m;
constexpr units::meter_t kWheelCircumference =
    kWheelDiameter * std::numbers::pi;
// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
// teeth on the bevel pinion
constexpr double kDrivingMotorReduction =
    (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
constexpr double kDriveWheelFreeSpeedRps =
    (kDrivingMotorFreeSpeedRps * kWheelCircumference.value()) /
    kDrivingMotorReduction;

constexpr double kDrivingEncoderPositionFactor =
    (kWheelDiameter.value() * std::numbers::pi) /
    kDrivingMotorReduction;  // meters
constexpr double kDrivingEncoderVelocityFactor =
    ((kWheelDiameter.value() * std::numbers::pi) / kDrivingMotorReduction) /
    60.0;  // meters per second

constexpr double kTurningEncoderPositionFactor =
    (2 * std::numbers::pi);  // radians
constexpr double kTurningEncoderVelocityFactor =
    (2 * std::numbers::pi) / 60.0;  // radians per second

constexpr units::radian_t kTurningEncoderPositionPIDMinInput = 0_rad;
constexpr units::radian_t kTurningEncoderPositionPIDMaxInput =
    units::radian_t{kTurningEncoderPositionFactor};

constexpr double kDrivingP = 0.04;
constexpr double kDrivingI = 0;
constexpr double kDrivingD = 0;
constexpr double kDrivingFF = (1 / kDriveWheelFreeSpeedRps);
constexpr double kDrivingMinOutput = -1;
constexpr double kDrivingMaxOutput = 1;

constexpr double kTurningP = 1;
constexpr double kTurningI = 0;
constexpr double kTurningD = 0;
constexpr double kTurningFF = 0;
constexpr double kTurningMinOutput = -1;
constexpr double kTurningMaxOutput = 1;

constexpr rev::CANSparkMax::IdleMode kDrivingMotorIdleMode =
    rev::CANSparkMax::IdleMode::kBrake;
constexpr rev::CANSparkMax::IdleMode kTurningMotorIdleMode =
    rev::CANSparkMax::IdleMode::kBrake;

constexpr units::ampere_t kDrivingMotorCurrentLimit = 50_A;
constexpr units::ampere_t kTurningMotorCurrentLimit = 20_A;
#endif
}  // namespace ModuleConstants

namespace AutoConstants {

// Only Constants here are the PID constants. Look in path planner for max veleocty/acceleration constants
// PID Constants for the tranlation (X and Y movement) of the robot during auto
constexpr double kPTanslationController = 6.0;
constexpr double kITanslationController = 1.7;
constexpr double kDTanslationController = 0.0;

// PID Constants for the rotation, or Yaw of the robot
constexpr double kPRotationController = 5.0;
constexpr double kIRotationController = 0.0;
constexpr double kDRotationController = 0.0;

}  // namespace AutoConstants

namespace ControllerConstants{

// Controller Constants for the flight elite drive controller

// Axis indexes
constexpr int kDriveLeftYIndex = 2; // An imput UP creates a NEGATIVE output
constexpr int kDriveLeftXIndex = 4; // An imput RIGHT creates a NEGATIVE output
constexpr int kDriveRightYIndex = 1; // An imput UP creates a NEGATIVE output
constexpr int kDriveRightXIndex = 0; // An imput RIGHT creates a NEGATIVE output

// Button/Switch indexes
constexpr int kFieldRelativeSwitchIndex = 1;
constexpr int kParkSwitchIndex = 2;
constexpr int kSlowStateSwitchIndex = 5;
constexpr int kResetGyroButtonIndex = 3;

} // namespace ControllerConstants

namespace IOConstants {
constexpr int kDriverControllerPort = 0;
}  // namespace OIConstants

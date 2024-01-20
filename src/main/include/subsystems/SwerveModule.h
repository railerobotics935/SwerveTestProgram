// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/AnalogInput.h>                        // for closed-loop control on RoboRIO
#include <frc/controller/PIDController.h>           // for closed-loop control on RoboRIO
#include <frc/controller/SimpleMotorFeedforward.h>  // for closed-loop control on RoboRIO
#include <frc/controller/ProfiledPIDController.h>   // for closed-loop control on RoboRIO

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkAbsoluteEncoder.h>               // for through-hole encoder on Spark MAX
#include <rev/SparkPIDController.h>                 // for closed-loop control on Spark MAX 
#include <rev/SparkRelativeEncoder.h>               // for closed-loop control on Spark MAX

#include <rev/SparkAnalogSensor.h>                  // for analog angle sensor on Spark MAX

#include "Constants.h"

class SwerveModule {
  public:
    SwerveModule(int driveMotorPort, int turningMotorPort,
                 const int turningEncoderPort, const double turningEncoderOffset);

  frc::SwerveModuleState GetState();

  frc::SwerveModulePosition GetPosition();

  void SetDesiredState(const frc::SwerveModuleState& state);

  // void ResetEncoders(); currently using absolute encoders, so you can't reset them digitaly

private:
  // We have to use meters here instead of radians due to the fact that
  // ProfiledPIDController's constraints only take in meters per second and
  // meters per second squared.
  
  rev::CANSparkMax m_drivingSparkMax;
  rev::CANSparkMax m_turningSparkMax;
  
  rev::SparkRelativeEncoder m_drivingEncoder =
      m_drivingSparkMax.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
#ifdef USE_SPARK_MAX_ANALOG_INPUT          // Absolute Angle Sensor on Spark MAX analog input
  rev::SparkAnalogSensor m_turningAbsoluteEncoder =
      m_turningSparkMax.GetAnalog(rev::SparkAnalogSensor::Mode::kAbsolute);
#endif
#ifdef USE_SPARK_MAX_ABS_ENCODER_INPUT     // Through-hole Encoder on Spark MAX frequency-pwm input
  rev::SparkAbsoluteEncoder m_turningAbsoluteEncoder =
      m_turningSparkMax.GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle);
#endif

#ifdef USE_ROBORIO_ANALOG_INPUT            // Absolute Angle Sensor on RoboRIO analog input
  frc::AnalogInput m_turningEncoder;
#endif

  double m_kTurningEncoderOffset;
  
#ifdef USE_ROBORIO_ANALOG_INPUT            // Absolute Angle Sensor on RoboRIO analog input
  frc::PIDController m_drivePIDController{ModuleConstants::kPModuleDriveController, ModuleConstants::kIModuleDriveController, ModuleConstants::kDModuleDriveController};
  frc::ProfiledPIDController<units::radians> m_turningPIDController{12.5, 190.0, 0.15,
      {ModuleConstants::kModuleMaxAngularVelocity, ModuleConstants::kModuleMaxAngularAcceleration}};
  frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{1_V, 2_V / 1_mps};
#endif
#ifdef USE_SPARK_MAX_ANALOG_INPUT          // Absolute Angle Sensor on Spark MAX analog input
  rev::SparkPIDController m_drivingPIDController = m_drivingSparkMax.GetPIDController();
  rev::SparkPIDController m_turningPIDController = m_turningSparkMax.GetPIDController();

  double m_chassisAngularOffset = 0;
  frc::SwerveModuleState m_desiredState{units::meters_per_second_t{0.0}, frc::Rotation2d()};
#endif
#ifdef USE_SPARK_MAX_ABS_ENCODER_INPUT     // Through-hole Encoder on Spark MAX frequency-pwm input
  rev::SparkPIDController m_drivingPIDController = m_drivingSparkMax.GetPIDController();
  rev::SparkPIDController m_turningPIDController = m_turningSparkMax.GetPIDController();

  double m_chassisAngularOffset = 0;
  frc::SwerveModuleState m_desiredState{units::meters_per_second_t{0.0}, frc::Rotation2d()};
#endif
};

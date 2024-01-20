// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"

#include <frc/geometry/Rotation2d.h>

#include <numbers>

#include "Constants.h"

using namespace ModuleConstants;

SwerveModule::SwerveModule(const int drivingCANId, const int turningCANId, 
                        int turningEncoderPort, 
                        double turningEncoderOffset)
    : m_drivingSparkMax(drivingCANId, rev::CANSparkMax::MotorType::kBrushless),
      m_turningSparkMax(turningCANId, rev::CANSparkMax::MotorType::kBrushless){
//      m_driveEncoder(m_driveMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)),
//      m_turningEncoder(turningEncoderPort){
#ifdef USE_ROBORIO_ANALOG_INPUT            // Absolute Angle Sensor on RoboRIO analog input
  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution. Don't forget the gear ratio of the swerve drive module.
  m_driveEncoder.SetVelocityConversionFactor(ModuleConstants::kDriveVelocityEncoderConversionFactor);
  m_driveEncoder.SetPositionConversionFactor(ModuleConstants::kDrivePositionEncoderConversionFactor);

  // Invert Turing motor so it turns CCW for positive voltage
  m_driveMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_turningMotor.SetInverted(true);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(
      units::radian_t{-std::numbers::pi}, units::radian_t{std::numbers::pi});
  
  m_kTurningEncoderOffset = turningEncoderOffset;
#endif

#ifdef USE_SPARK_MAX_ANALOG_INPUT          // Absolute Angle Sensor on Spark MAX analog input
  // Factory reset, so we get the SPARKS MAX to a known state before configuring
  // them. This is useful in case a SPARK MAX is swapped out.
  m_drivingSparkMax.RestoreFactoryDefaults();
  m_turningSparkMax.RestoreFactoryDefaults();

  // Apply position and velocity conversion factors for the driving encoder. The
  // native units for position and velocity are rotations and RPM, respectively,
  // but we want meters and meters per second to use with WPILib's swerve APIs.
  m_drivingEncoder.SetPositionConversionFactor(kDrivingEncoderPositionFactor);
  m_drivingEncoder.SetVelocityConversionFactor(kDrivingEncoderVelocityFactor);

  // Apply position and velocity conversion factors for the turning encoder. We
  // want these in radians and radians per second to use with WPILib's swerve
  // APIs.
  m_turningAbsoluteEncoder.SetPositionConversionFactor(kTurningEncoderPositionFactor);
  m_turningAbsoluteEncoder.SetVelocityConversionFactor(kTurningEncoderVelocityFactor);

  // Invert the turning encoder, since the output shaft rotates in the opposite
  // direction of the steering motor in the MAXSwerve Module.
  m_turningAbsoluteEncoder.SetInverted(kTurningEncoderInverted);

  // Enable PID wrap around for the turning motor. This will allow the PID
  // controller to go through 0 to get to the setpoint i.e. going from 350
  // degrees to 10 degrees will go through 0 rather than the other direction
  // which is a longer route.
  m_turningPIDController.SetPositionPIDWrappingEnabled(true);
  m_turningPIDController.SetPositionPIDWrappingMinInput(kTurningEncoderPositionPIDMinInput.value());
  m_turningPIDController.SetPositionPIDWrappingMaxInput(kTurningEncoderPositionPIDMaxInput.value());
        
  // Set the PID Controller to use the duty cycle encoder on the swerve
  // module instead of the built in NEO550 encoder.
  m_turningPIDController.SetFeedbackDevice(m_turningAbsoluteEncoder);

  // Set the PID gains for the driving motor. Note these are example gains, and
  // you may need to tune them for your own robot!
  m_drivingPIDController.SetP(kDrivingP);
  m_drivingPIDController.SetI(kDrivingI);
  m_drivingPIDController.SetD(kDrivingD);
  m_drivingPIDController.SetFF(kDrivingFF);
  m_drivingPIDController.SetOutputRange(kDrivingMinOutput, kDrivingMaxOutput);

  // Set the PID gains for the turning motor. Note these are example gains, and
  // you may need to tune them for your own robot!
  m_turningPIDController.SetP(kTurningP);
  m_turningPIDController.SetI(kTurningI);
  m_turningPIDController.SetD(kTurningD);
  m_turningPIDController.SetFF(kTurningFF);
  m_turningPIDController.SetOutputRange(kTurningMinOutput, kTurningMaxOutput);

  m_drivingSparkMax.SetIdleMode(kDrivingMotorIdleMode);
  m_turningSparkMax.SetIdleMode(kTurningMotorIdleMode);
  m_drivingSparkMax.SetSmartCurrentLimit(kDrivingMotorCurrentLimit.value());
  m_turningSparkMax.SetSmartCurrentLimit(kDrivingMotorCurrentLimit.value());

  // Save the SPARK MAX configurations. If a SPARK MAX browns out during
  // operation, it will maintain the above configurations.
  m_drivingSparkMax.BurnFlash();
  m_turningSparkMax.BurnFlash();

//  m_chassisAngularOffset = chassisAngularOffset;
  m_chassisAngularOffset = 0.0;
  m_desiredState.angle = frc::Rotation2d(units::radian_t{m_turningAbsoluteEncoder.GetPosition()});
  m_drivingEncoder.SetPosition(0);
#endif

#ifdef USE_SPARK_MAX_ABS_ENCODER_INPUT     // Through-hole Encoder on Spark MAX frequency-pwm input
  // Factory reset, so we get the SPARKS MAX to a known state before configuring
  // them. This is useful in case a SPARK MAX is swapped out.
  m_drivingSparkMax.RestoreFactoryDefaults();
  m_turningSparkMax.RestoreFactoryDefaults();

  // Apply position and velocity conversion factors for the driving encoder. The
  // native units for position and velocity are rotations and RPM, respectively,
  // but we want meters and meters per second to use with WPILib's swerve APIs.
  m_drivingEncoder.SetPositionConversionFactor(kDrivingEncoderPositionFactor);
  m_drivingEncoder.SetVelocityConversionFactor(kDrivingEncoderVelocityFactor);

  // Apply position and velocity conversion factors for the turning encoder. We
  // want these in radians and radians per second to use with WPILib's swerve
  // APIs.
  m_turningAbsoluteEncoder.SetPositionConversionFactor(kTurningEncoderPositionFactor);
  m_turningAbsoluteEncoder.SetVelocityConversionFactor(kTurningEncoderVelocityFactor);

  // Invert the turning encoder, since the output shaft rotates in the opposite
  // direction of the steering motor in the MAXSwerve Module.
  m_turningAbsoluteEncoder.SetInverted(kTurningEncoderInverted);

  // Enable PID wrap around for the turning motor. This will allow the PID
  // controller to go through 0 to get to the setpoint i.e. going from 350
  // degrees to 10 degrees will go through 0 rather than the other direction
  // which is a longer route.
  m_turningPIDController.SetPositionPIDWrappingEnabled(true);
  m_turningPIDController.SetPositionPIDWrappingMinInput(kTurningEncoderPositionPIDMinInput.value());
  m_turningPIDController.SetPositionPIDWrappingMaxInput(kTurningEncoderPositionPIDMaxInput.value());
        
  // Set the PID Controller to use the duty cycle encoder on the swerve
  // module instead of the built in NEO550 encoder.
  m_turningPIDController.SetFeedbackDevice(m_turningAbsoluteEncoder);

  // Set the PID gains for the driving motor. Note these are example gains, and
  // you may need to tune them for your own robot!
  m_drivingPIDController.SetP(kDrivingP);
  m_drivingPIDController.SetI(kDrivingI);
  m_drivingPIDController.SetD(kDrivingD);
  m_drivingPIDController.SetFF(kDrivingFF);
  m_drivingPIDController.SetOutputRange(kDrivingMinOutput, kDrivingMaxOutput);

  // Set the PID gains for the turning motor. Note these are example gains, and
  // you may need to tune them for your own robot!
  m_turningPIDController.SetP(kTurningP);
  m_turningPIDController.SetI(kTurningI);
  m_turningPIDController.SetD(kTurningD);
  m_turningPIDController.SetFF(kTurningFF);
  m_turningPIDController.SetOutputRange(kTurningMinOutput, kTurningMaxOutput);

  m_drivingSparkMax.SetIdleMode(kDrivingMotorIdleMode);
  m_turningSparkMax.SetIdleMode(kTurningMotorIdleMode);
  m_drivingSparkMax.SetSmartCurrentLimit(kDrivingMotorCurrentLimit.value());
  m_turningSparkMax.SetSmartCurrentLimit(kDrivingMotorCurrentLimit.value());

  // Save the SPARK MAX configurations. If a SPARK MAX browns out during
  // operation, it will maintain the above configurations.
  m_drivingSparkMax.BurnFlash();
  m_turningSparkMax.BurnFlash();

//  m_chassisAngularOffset = chassisAngularOffset;
  m_chassisAngularOffset = 0.0;
  m_desiredState.angle = frc::Rotation2d(units::radian_t{m_turningAbsoluteEncoder.GetPosition()});
  m_drivingEncoder.SetPosition(0);
#endif
}

frc::SwerveModuleState SwerveModule::GetState() {
#ifdef USE_ROBORIO_ANALOG_INPUT
  return {units::meters_per_second_t{m_driveEncoder.GetVelocity()},
          units::radian_t{(m_turningEncoder.GetVoltage() * ModuleConstants::ANALOG_TO_RAD_FACTOR) - m_kTurningEncoderOffset}};
#endif
#ifdef USE_SPARK_MAX_ANALOG_INPUT
  return {units::meters_per_second_t{m_drivingEncoder.GetVelocity()},
          units::radian_t{m_turningAbsoluteEncoder.GetPosition() - m_chassisAngularOffset}};
//  return {units::meters_per_second_t{m_driveEncoder.GetVelocity()},
//          units::radian_t{(m_turningMotor.GetAnalog(rev::SparkAnalogSensor::Mode::kAbsolute).GetVoltage() * ModuleConstants::ANALOG_TO_RAD_FACTOR) - m_kTurningEncoderOffset}};
#endif
#ifdef USE_SPARK_MAX_ABS_ENCODER_INPUT
  return {units::meters_per_second_t{m_drivingEncoder.GetVelocity()},
          units::radian_t{m_turningAbsoluteEncoder.GetPosition() - m_chassisAngularOffset}};
#endif
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
#ifdef USE_ROBORIO_ANALOG_INPUT
  return {units::meter_t{m_driveEncoder.GetPosition()},
          units::radian_t{(m_turningEncoder.GetVoltage() * ModuleConstants::ANALOG_TO_RAD_FACTOR) - m_kTurningEncoderOffset}};
#endif
#ifdef USE_SPARK_MAX_ANALOG_INPUT
//  return {units::meter_t{m_driveEncoder.GetPosition()},
//          units::radian_t{(m_turningMotor.GetAnalog(rev::SparkAnalogSensor::Mode::kAbsolute).GetVoltage() * ModuleConstants::ANALOG_TO_RAD_FACTOR) - m_kTurningEncoderOffset}};
  return {units::meter_t{m_drivingEncoder.GetPosition()},
          units::radian_t{m_turningAbsoluteEncoder.GetPosition() - m_chassisAngularOffset}};
#endif
#ifdef USE_SPARK_MAX_ABS_ENCODER_INPUT
  return {units::meter_t{m_drivingEncoder.GetPosition()},
          units::radian_t{m_turningAbsoluteEncoder.GetPosition() - m_chassisAngularOffset}};
#endif
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& desiredState) {
#ifdef USE_ROBORIO_ANALOG_INPUT            // Absolute Angle Sensor on RoboRIO analog input
  // Optimize the reference state to avoid spinning further than 90 degrees
  const auto state = frc::SwerveModuleState::Optimize(
      referenceState, GetState().angle.Radians());

  // Calculate the drive output from the drive PID controller.
  const auto driveOutput = m_drivePIDController.Calculate(
      m_driveEncoder.GetVelocity(), state.speed.value());

  // Calculate the turning motor output from the turning PID controller.
  auto turnOutput = m_turningPIDController.Calculate(
      GetState().angle.Radians(), state.angle.Radians());

  const auto driveFeedforward = m_driveFeedforward.Calculate(state.speed);

  // Set the motor outputs.
  m_driveMotor.SetVoltage(units::volt_t{driveOutput} + units::volt_t{driveFeedforward});
  m_turningMotor.SetVoltage(units::volt_t{turnOutput});
#endif
#ifdef USE_SPARK_MAX_ANALOG_INPUT
  // Apply chassis angular offset to the desired state.
  frc::SwerveModuleState correctedDesiredState{};
  correctedDesiredState.speed = desiredState.speed;
  correctedDesiredState.angle =
      desiredState.angle +
      frc::Rotation2d(units::radian_t{m_chassisAngularOffset});

  // Optimize the reference state to avoid spinning further than 90 degrees.
  frc::SwerveModuleState optimizedDesiredState{frc::SwerveModuleState::Optimize(
      correctedDesiredState, frc::Rotation2d(units::radian_t{
                                 m_turningAbsoluteEncoder.GetPosition()}))};

  // Command driving and turning SPARKS MAX towards their respective setpoints.
  m_drivingPIDController.SetReference((double)optimizedDesiredState.speed,
                                      rev::CANSparkMax::ControlType::kVelocity);
  m_turningPIDController.SetReference(
      optimizedDesiredState.angle.Radians().value(),
      rev::CANSparkMax::ControlType::kPosition);

  m_desiredState = desiredState;
#endif
#ifdef USE_SPARK_MAX_ABS_ENCODER_INPUT
  // Apply chassis angular offset to the desired state.
  frc::SwerveModuleState correctedDesiredState{};
  correctedDesiredState.speed = desiredState.speed;
  correctedDesiredState.angle =
      desiredState.angle +
      frc::Rotation2d(units::radian_t{m_chassisAngularOffset});

  // Optimize the reference state to avoid spinning further than 90 degrees.
  frc::SwerveModuleState optimizedDesiredState{frc::SwerveModuleState::Optimize(
      correctedDesiredState, frc::Rotation2d(units::radian_t{
                                 m_turningAbsoluteEncoder.GetPosition()}))};

  // Command driving and turning SPARKS MAX towards their respective setpoints.
  m_drivingPIDController.SetReference((double)optimizedDesiredState.speed,
                                      rev::CANSparkMax::ControlType::kVelocity);
  m_turningPIDController.SetReference(
      optimizedDesiredState.angle.Radians().value(),
      rev::CANSparkMax::ControlType::kPosition);

  m_desiredState = desiredState;
#endif
}

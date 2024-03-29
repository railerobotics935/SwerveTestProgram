// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/DriverStation.h>

#include <iostream>
#include "Constants.h"

using namespace DriveConstants;


DriveSubsystem::DriveSubsystem()
  : m_frontLeft{kFrontLeftDriveMotorPort,
                kFrontLeftTurningMotorPort,
                kFrontLeftTurningEncoderPort,
                kFrontLeftDriveEncoderOffset},

    m_frontRight{
        kFrontRightDriveMotorPort,       
        kFrontRightTurningMotorPort,
        kFrontRightTurningEncoderPort,
        kFrontRightDriveEncoderOffset},
    
    m_backLeft{
        kBackLeftDriveMotorPort,       
        kBackLeftTurningMotorPort,
        kBackLeftTurningEncoderPort,
        kBackLeftDriveEncoderOffset},

    m_backRight{
        kBackRightDriveMotorPort,       
        kBackRightTurningMotorPort,  
        kBackRightTurningEncoderPort,
        kBackRightDriveEncoderOffset},

    m_odometry{m_driveKinematics,
                m_gyro.GetAngle(frc::ADIS16470_IMU::kYaw),
                {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                m_backLeft.GetPosition(), m_backRight.GetPosition()},
                frc::Pose2d{(units::meter_t)3.0, (units::meter_t)3.0, m_gyro.GetAngle(frc::ADIS16470_IMU::kYaw)}} 
{

  // Initialize shuffleboard communication
  auto nt_inst = nt::NetworkTableInstance::GetDefault();
  auto nt_table = nt_inst.GetTable("datatable");

  nte_fl_set_angle = nt_table->GetEntry("Swerve Drive/Front Left/Set Angle");
  nte_fr_set_angle = nt_table->GetEntry("Swerve Drive/Front Right/Set Angle");
  nte_bl_set_angle = nt_table->GetEntry("Swerve Drive/Back Left/Set Angle");
  nte_br_set_angle = nt_table->GetEntry("Swerve Drive/Back Right/Set Angle");
  nte_fl_set_speed = nt_table->GetEntry("Swerve Drive/Front Left/Set Speed");
  nte_fr_set_speed = nt_table->GetEntry("Swerve Drive/Front Right/Set Speed");
  nte_bl_set_speed = nt_table->GetEntry("Swerve Drive/Back Left/Set Speed");
  nte_br_set_speed = nt_table->GetEntry("Swerve Drive/Back Right/Set Speed");

  nte_fl_real_angle = nt_table->GetEntry("Swerve Drive/Front Left/Real Angle");
  nte_fr_real_angle = nt_table->GetEntry("Swerve Drive/Front Right/Real Angle");
  nte_bl_real_angle = nt_table->GetEntry("Swerve Drive/Back Left/Real Angle");
  nte_br_real_angle = nt_table->GetEntry("Swerve Drive/Back Right/Real Angle");
  nte_fl_real_speed = nt_table->GetEntry("Swerve Drive/Front Left/Real Speed");
  nte_fr_real_speed = nt_table->GetEntry("Swerve Drive/Front Right/Real Speed");
  nte_bl_real_speed = nt_table->GetEntry("Swerve Drive/Back Left/Real Speed");
  nte_br_real_speed = nt_table->GetEntry("Swerve Drive/Back Right/Real Speed");

  nte_fl_raw_encoder_voltage = nt_table->GetEntry("Swerve Drive/Front Left/Encoder Voltage");
  nte_fr_raw_encoder_voltage = nt_table->GetEntry("Swerve Drive/Front Right/Encoder Voltage");
  nte_bl_raw_encoder_voltage = nt_table->GetEntry("Swerve Drive/Back Left/Encoder Voltage");
  nte_br_raw_encoder_voltage = nt_table->GetEntry("Swerve Drive/Back Right/Encoder Voltage");

  nte_gyro_angle = nt_table->GetEntry("Swerve Drive/Gyro Angle");
  nte_robot_x = nt_table->GetEntry("Swerve Drive/Robot X");
  nte_robot_y = nt_table->GetEntry("Swerve Drive/Robot Y");

  // Send Field to shuffleboard
  frc::Shuffleboard::GetTab("Field").Add(m_field);

}

// Returns true is the allience selected is red
bool DriveSubsystem::InRedAlience() {
  if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
    return false;
  else
    return true;
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(m_gyro.GetAngle(frc::ADIS16470_IMU::kYaw),
                    {m_frontLeft.GetPosition(), m_frontRight.GetPosition(), 
                    m_backLeft.GetPosition(), m_backRight.GetPosition()});
                    
  nte_fl_real_angle.SetDouble((double)m_frontLeft.GetState().angle.Radians());
  nte_fr_real_angle.SetDouble((double)m_frontRight.GetState().angle.Radians());
  nte_bl_real_angle.SetDouble((double)m_backLeft.GetState().angle.Radians());
  nte_br_real_angle.SetDouble((double)m_backRight.GetState().angle.Radians());
  nte_fl_real_speed.SetDouble((double)m_frontLeft.GetState().speed);
  nte_fr_real_speed.SetDouble((double)m_frontRight.GetState().speed);
  nte_bl_real_speed.SetDouble((double)m_backLeft.GetState().speed);
  nte_br_real_speed.SetDouble((double)m_backRight.GetState().speed);

  nte_gyro_angle.SetDouble((double)m_odometry.GetPose().Rotation().Radians());
  nte_robot_x.SetDouble((double)m_odometry.GetPose().X());
  nte_robot_y.SetDouble((double)m_odometry.GetPose().Y());

  //nte_fl_raw_encoder_voltage.SetDouble(m_frontLeft.GetEncoderVoltage());
  //nte_fr_raw_encoder_voltage.SetDouble(m_frontRight.GetEncoderVoltage());
  //nte_bl_raw_encoder_voltage.SetDouble(m_backLeft.GetEncoderVoltage());
  //nte_br_raw_encoder_voltage.SetDouble(m_backRight.GetEncoderVoltage());

  // Set robot possition wto shuffleboard field
  m_field.SetRobotPose(m_odometry.GetPose());

}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative) {
  auto states = m_driveKinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, m_gyro.GetAngle(frc::ADIS16470_IMU::kYaw))
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  m_driveKinematics.DesaturateWheelSpeeds(&states, ModuleConstants::kModuleMaxLinearVelocity);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);

  // Network table entries
  nte_fl_set_angle.SetDouble((double)fl.angle.Radians());
  nte_fr_set_angle.SetDouble((double)fr.angle.Radians());
  nte_bl_set_angle.SetDouble((double)bl.angle.Radians());
  nte_br_set_angle.SetDouble((double)br.angle.Radians());
  nte_fl_set_speed.SetDouble((double)fl.speed);
  nte_fr_set_speed.SetDouble((double)fr.speed);
  nte_bl_set_speed.SetDouble((double)bl.speed);
  nte_br_set_speed.SetDouble((double)br.speed);
  
}

void DriveSubsystem::DriveWithChassisSpeeds(frc::ChassisSpeeds speeds) {
  auto states = m_driveKinematics.ToSwerveModuleStates(speeds);

  m_driveKinematics.DesaturateWheelSpeeds(&states, ModuleConstants::kModuleMaxLinearVelocity);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  m_driveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         ModuleConstants::kModuleMaxLinearVelocity);
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_backLeft.SetDesiredState(desiredStates[2]);
  m_backRight.SetDesiredState(desiredStates[3]);
}

// Method to put the Robot in Park
void DriveSubsystem::Park()
{
  frc::SwerveModuleState fl;
  frc::SwerveModuleState fr;
  frc::SwerveModuleState bl;
  frc::SwerveModuleState br;

  fl.angle = frc::Rotation2d (units::radian_t(std::numbers::pi / 4));
  fr.angle = frc::Rotation2d (units::radian_t(-std::numbers::pi / 4));
  bl.angle = frc::Rotation2d (units::radian_t(-std::numbers::pi / 4));
  br.angle = frc::Rotation2d (units::radian_t(std::numbers::pi / 4));

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);
}

units::degree_t DriveSubsystem::GetHeading() const {
  return m_gyro.GetAngle(frc::ADIS16470_IMU::kYaw);
}

void DriveSubsystem::ZeroHeading() {
  m_gyro.Reset();
}

double DriveSubsystem::GetTurnRate() {
  return (double)-m_gyro.GetRate(frc::ADIS16470_IMU::kYaw);
}

frc::Pose2d DriveSubsystem::GetPose() {
  return m_odometry.GetPose();
}

frc::ChassisSpeeds DriveSubsystem::GetRobotRelativeSpeeds()
{
  return m_driveKinematics.ToChassisSpeeds(m_frontLeft.GetState(), 
                                           m_frontRight.GetState(),
                                           m_backLeft.GetState(),
                                           m_backRight.GetState());
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(
      GetHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_backLeft.GetPosition(), m_backRight.GetPosition()},
      pose);
}

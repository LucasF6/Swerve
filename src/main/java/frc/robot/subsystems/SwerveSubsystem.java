// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;

import static frc.robot.Constants.Swerve.*;

import com.ctre.phoenix.sensors.Pigeon2;

public class SwerveSubsystem extends SubsystemBase {
  private SwerveModule m_frontRightModule = new SwerveModule(
      FR_DRIVE_ID, FR_STEER_ID, FR_ENCODER_ID, 
      FR_DRIVE_INVERTED, FR_STEER_INVERTED, FR_OFFSET_DEGREES);
  private SwerveModule m_frontLeftModule = new SwerveModule(
      FL_DRIVE_ID, FL_STEER_ID, FL_ENCODER_ID, 
      FL_DRIVE_INVERTED, FL_STEER_INVERTED, FL_OFFSET_DEGREES);
  private SwerveModule m_backRightModule = new SwerveModule(
      BR_DRIVE_ID, BR_STEER_ID, BR_ENCODER_ID, 
      BR_DRIVE_INVERTED, BR_STEER_INVERTED, BR_OFFSET_DEGREES);
  private SwerveModule m_backLeftModule = new SwerveModule(
      BL_DRIVE_ID, BL_STEER_ID, BL_ENCODER_ID, 
      BL_DRIVE_INVERTED, BL_STEER_INVERTED, BL_OFFSET_DEGREES);

  private SwerveModule[] m_modules = new SwerveModule[] {
    new SwerveModule(
        FR_DRIVE_ID, FR_STEER_ID, FR_ENCODER_ID, 
        FR_DRIVE_INVERTED, FR_STEER_INVERTED, FR_OFFSET_DEGREES),
    new SwerveModule(
        FL_DRIVE_ID, FL_STEER_ID, FL_ENCODER_ID, 
        FL_DRIVE_INVERTED, FL_STEER_INVERTED, FL_OFFSET_DEGREES),
    new SwerveModule(
        BR_DRIVE_ID, BR_STEER_ID, BR_ENCODER_ID, 
        BR_DRIVE_INVERTED, BR_STEER_INVERTED, BR_OFFSET_DEGREES),
    new SwerveModule(
        BL_DRIVE_ID, BL_STEER_ID, BL_ENCODER_ID, 
        BL_DRIVE_INVERTED, BL_STEER_INVERTED, BL_OFFSET_DEGREES)};

  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      FR_LOCATION,
      FL_LOCATION,
      BR_LOCATION,
      BL_LOCATION);

  private Pigeon2 m_gyro = new Pigeon2(20);
  private Pose2d m_pose = new Pose2d();
  private SwerveModulePosition[] m_modulePositions = new SwerveModulePosition[4];

  private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
    m_kinematics,
    Rotation2d.fromDegrees(m_gyro.getYaw()),
    m_modulePositions,
    m_pose);

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for (int i = 0; i < 4; i++) {
      m_modulePositions[i] = m_modules[i].getModulePosition();
    }
    m_pose = m_odometry.update(null, m_modulePositions);
  }

  public void driveRobotOriented(ChassisSpeeds speeds) {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);

    for(int i = 0; i < 4; i++) {
      m_modules[i].setState(SwerveModuleState.optimize(
          states[i], m_modules[i].getAngleMeasurement()));
    }
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(m_gyro.getYaw());
  }

}

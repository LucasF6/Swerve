// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DefaultDrive;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private XboxController m_controller = new XboxController(1);

  private SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

  private DefaultDrive m_defaultDrive = new DefaultDrive(
    m_swerveSubsystem,
    m_controller::getLeftY,
    m_controller::getLeftX,
    m_controller::getRightX);

  public RobotContainer() {
    m_swerveSubsystem.setDefaultCommand(m_defaultDrive);

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

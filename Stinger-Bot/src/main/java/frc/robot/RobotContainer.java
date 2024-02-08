// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.SwerveBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.TeleopSwerve;

public class RobotContainer {
  //Controllers
  private final CommandXboxController driverController = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);

  private final SwerveBase swerveBase;
  private final TeleopSwerve teleopSwerve;

  public RobotContainer() {

    //initialize subsytems
    swerveBase = new SwerveBase();

    teleopSwerve = new TeleopSwerve(
      swerveBase,
      //inverts controls because joysticks are back-right (+) while robot is front-left (+)
      () -> -driverController.getLeftY(),
      () -> -driverController.getLeftX(),
      () -> -driverController.getRightX(),
      () -> driverController.rightBumper().getAsBoolean()
      );
    
    swerveBase.setDefaultCommand(teleopSwerve);

    configureBindings();
  }

  //Use for Mapping button bindings
  private void configureBindings() {
    //driver buttton to zero gyro
    driverController.y().onTrue(new InstantCommand(() -> swerveBase.zeroGyro()));

    
  
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.SwerveBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.TeleopSwerve;
import edu.wpi.first.wpilibj.XboxController;

public class RobotContainer {
  //declare Controllers
  private final XboxController driverController;
  //private final XboxController operatorController;

  private final double translationAxis;
  private final double strafeAxis;
  private final double rotationAxis;

  private final boolean robotCentric;


  private final SwerveBase swerveBase;
  private final TeleopSwerve teleopSwerve;

  public RobotContainer() {
    //initialize controllers
    driverController = new XboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
    //operatorController = new XboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);

    //initialize driver controller inputs
    translationAxis = -driverController.getLeftY();
    strafeAxis = driverController.getLeftX();
    rotationAxis = driverController.getRightX();

    //initialize drive button inputs

    //Zero gyro button?
    
    robotCentric = driverController.getLeftBumper();

    //initialize subsytems
    swerveBase = new SwerveBase();
 
    teleopSwerve = new TeleopSwerve(
      swerveBase, 
      () -> MathUtil.applyDeadband(translationAxis, ControllerConstants.DEADBANDRANGE),
      () -> MathUtil.applyDeadband(strafeAxis, ControllerConstants.DEADBANDRANGE),
      () -> MathUtil.applyDeadband(rotationAxis, ControllerConstants.DEADBANDRANGE),
      () -> robotCentric);

    
    swerveBase.setDefaultCommand(teleopSwerve);

    configureBindings();
  }

  //Use for Mapping button bindings
  private void configureBindings() {
    //driver buttton to zero gyro
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

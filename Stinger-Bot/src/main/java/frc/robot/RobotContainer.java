// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.SwerveBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.TeleopSwerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class RobotContainer {
  //declare Controllers
  private final Joystick driverController = new Joystick(ControllerConstants.DRIVER_CONTROLLER_PORT);
  //private final XboxController operatorController;

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final JoystickButton robotCentric =
      new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);

  private final SwerveBase swerveBase;
  private final TeleopSwerve teleopSwerve;

  public RobotContainer() {
    //initialize drive button inputs

    //Zero gyro button?

    //initialize subsytems
    swerveBase = new SwerveBase();
 
    teleopSwerve = new TeleopSwerve(
      swerveBase, 
      () -> -driverController.getRawAxis(translationAxis),
      () -> driverController.getRawAxis(strafeAxis),
      () -> driverController.getRawAxis(rotationAxis),
      () -> robotCentric.getAsBoolean());

    
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.TransferConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.drive.SwerveBase;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Shooter;
import frc.robot.subsystems.transfer.Transfer;

public class RobotContainer {
  //Controllers
  private final CommandXboxController driverController = new CommandXboxController(ControllerConstants.driverControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(ControllerConstants.operatorControllerPort);

  private final SwerveBase swerveBase;
  private final TeleopSwerve teleopSwerve;

  private final Intake intakeSub;

  private final Transfer transferSub;

  private final Shooter launcherSub;

  public RobotContainer() {

    //initialize subsytems
    swerveBase = new SwerveBase();

    transferSub = new Transfer();

    intakeSub = new Intake();

    teleopSwerve = new TeleopSwerve(
      swerveBase,
      //inverts controls because joysticks are back-right (+) while robot is front-left (+)
      () -> -driverController.getLeftY(),
      () -> -driverController.getLeftX(),
      () -> -driverController.getRightX(),
      () -> driverController.rightBumper().getAsBoolean()
      );
    
    swerveBase.setDefaultCommand(teleopSwerve);

    //test shooter kraken motors 02/17/2024
    launcherSub = new Shooter();
    

    configureBindings();
  }

/* 
  public Command resetGyro(){
    return new InstantCommand(() -> swerveBase.zeroGyro());
  }
*/
  //Use for Mapping button bindings
  private void configureBindings(){
    //driver buttton to zero gyro
    driverController.y().onTrue(new InstantCommand(() -> swerveBase.zeroGyro()));

    
    //operator controller bindings
    operatorController.rightBumper().onTrue(new InstantCommand(() -> transferSub.setTransferSpeed(TransferConstants.transferSeed)));
    operatorController.leftBumper().onTrue(new InstantCommand(() -> transferSub.setTransferSpeed(TransferConstants.tranSpitSpeed)));
    transferSub.setDefaultCommand(new InstantCommand(() -> transferSub.stopTransfer()));

    
    

    
  
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

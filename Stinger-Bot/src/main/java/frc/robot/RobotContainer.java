// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.TransferConstants;
import frc.robot.commands.IntakeNoteReg;
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
  private final Intake intakeSub;
  private final Transfer transferSub;
  private final Shooter launcherSub;

  private TeleopSwerve teleopSwerve;



  public RobotContainer() {

    //initialize subsytems
    swerveBase = new SwerveBase();

    intakeSub = new Intake();

    transferSub = new Transfer();

    launcherSub = new Shooter();

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

/* 
  public Command resetGyro(){
    return new InstantCommand(() -> swerveBase.zeroGyro());
  }
*/
  //Use for Mapping button bindings
  private void configureBindings(){
    //driver buttton to zero gyro
    driverController.y().onTrue(new InstantCommand(() -> swerveBase.zeroGyro()));

    
    //operator controller bindings for manual control and testing 
    /*add vision to determine speed and angle necessary to score in speaker */
    /*Arm, shooter, and transfer should eventually be in the same command for shooting */
    operatorController.rightTrigger().whileTrue(new InstantCommand(() -> launcherSub.setShooterSpeed(LauncherConstants.lowPositionSpeed,  LauncherConstants.ffOvercomeGrav)));
    operatorController.leftTrigger().whileTrue(new InstantCommand(() -> launcherSub.setShooterSpeed(-LauncherConstants.intakeFromShooterSpeed, LauncherConstants.ffOvercomeGrav)));
    operatorController.y().onTrue(new InstantCommand(() -> launcherSub.stopShooter()));

    operatorController.rightBumper().whileTrue(
      new IntakeNoteReg(
        intakeSub, 
        IntakeConstants.intakeSpeed, 
        transferSub, 
        TransferConstants.transferSeed));

      operatorController.leftBumper().whileTrue(
        new IntakeNoteReg(
          intakeSub, 
          IntakeConstants.intakeSpitSpeed, 
          transferSub, 
          TransferConstants.tranSpitSpeed));



    //intakeSub.setDefaultCommand(new InstantCommand(() -> intakeSub.stopIntake()));
    //transferSub.setDefaultCommand(new InstantCommand(() -> transferSub.stopTransfer()));

    //Arm
  
  
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

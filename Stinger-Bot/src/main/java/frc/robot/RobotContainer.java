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
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.TransferConstants;
import frc.robot.Util.Setpoints.GameState;
import frc.robot.commands.IntakeNoteReg;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.manualIntakeControl;
import frc.robot.commands.prepToShoot;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmDefault;
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
  private final Arm armSub;

  private TeleopSwerve teleopSwerve;



  public RobotContainer() {

    //initialize subsytems
    swerveBase = new SwerveBase();

    intakeSub = new Intake();

    transferSub = new Transfer();

    launcherSub = new Shooter();

    armSub = new Arm();

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
    //driver controls
    //driver buttton to zero gyro
    driverController.povUp().onTrue(new InstantCommand(() -> swerveBase.zeroGyro()));

    /* 
    //auto intake using photo sensor
    driverController.leftBumper().whileTrue(
      new IntakeNoteReg(
        intakeSub, 
        IntakeConstants.intakeSpeed, 
        transferSub, 
        TransferConstants.transferSeed));
    */
    //bring down arm to stow/intake position then run intake command until note is detected
    driverController.leftTrigger(0.4).onTrue(armSub.prepareForIntakeCommand()
            .andThen(new IntakeNoteReg(
              intakeSub, 
              IntakeConstants.intakeSpeed, 
              transferSub, 
              TransferConstants.transferSeed)));

    //Slow down robot whenm left stick held down
    //driverController.leftStick().whileTrue();

    /*
     * lock robot into positions N,E,S,W
     */

    
    //operator controller bindings for manual control and testing 
    /*add vision to determine speed and angle necessary to score in speaker */
    /* 
    operatorController.leftBumper().whileTrue(
      new IntakeNoteReg(
        intakeSub, 
        IntakeConstants.intakeSpitSpeed, 
        transferSub, 
        TransferConstants.tranSpitSpeed));
    */

    //manual shooter based on trigger axis value
    operatorController.rightTrigger().whileTrue(
      new InstantCommand(() -> launcherSub.setShooterSpeed(
        operatorController.getRightTriggerAxis() * 100, //*100 to go obtain a 0.1:10 ratio similar to that of Duty Cycle -> RPS
        LauncherConstants.ffOvercomeGrav)));//Note: only shoots

    //Feed Note to shooter
    operatorController.rightBumper().whileTrue(new InstantCommand(() -> transferSub.setTransferSpeed(TransferConstants.transferSeed)));

    //intake from shooter
    operatorController.leftBumper().whileTrue(
      new InstantCommand(() -> launcherSub.setShooterSpeed(LauncherConstants.intakeFromShooterSpeed, LauncherConstants.ffOvercomeGrav)));
    //saftey stop for launcher
    operatorController.povLeft().onTrue(new InstantCommand(() -> launcherSub.stopShooter()));


    //Arm related commands
    //Default command is manual arm movment using left bumber and left Y joystick axis
    armSub.setDefaultCommand(new ArmDefault(armSub, operatorController.leftStick(), ()-> -operatorController.getRightY()));

    //stow arm if not already
    operatorController.povRight().onTrue(new prepToShoot(RobotConstants.STOWED, armSub));
    //bindings to set arm and shooter setpoints
    /* 
    operatorController.y().onTrue(new prepToShoot(RobotConstants.AMP, armSub));
    operatorController.b().onTrue(new prepToShoot(RobotConstants.WING, armSub));
    operatorController.a().onTrue(new prepToShoot(RobotConstants.SPEAKER, armSub));
    operatorController.x().onTrue(new prepToShoot(RobotConstants.PODIUM, armSub));
    */
    +
    operatorController.y().onTrue(new prepToShoot(RobotConstants.AMP, armSub, launcherSub));
    operatorController.b().onTrue(new prepToShoot(RobotConstants.WING, armSub, launcherSub));
    operatorController.a().onTrue(new prepToShoot(RobotConstants.SPEAKER, armSub, launcherSub));
    operatorController.x().onTrue(new prepToShoot(RobotConstants.PODIUM, armSub, launcherSub));


    //Dpad up: manually intake note without photo sensor
    operatorController.povUp().whileTrue(
      new manualIntakeControl(
        intakeSub, IntakeConstants.intakeSpeed,
        transferSub, TransferConstants.transferSeed));

    //Dpad up: manually outtake note
    operatorController.povDown().whileTrue(
      new manualIntakeControl(
        intakeSub, IntakeConstants.intakeSpitSpeed,
        transferSub, TransferConstants.tranSpitSpeed));



  
  
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

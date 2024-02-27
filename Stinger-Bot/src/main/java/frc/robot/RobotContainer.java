// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
//import frc.robot.Util.Setpoints.GameState;
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
  private final Shooter shooterSub;
  private final Arm armSub;

  private TeleopSwerve teleopSwerve;



  public RobotContainer() {

    //initialize subsytems
    swerveBase = new SwerveBase();

    intakeSub = new Intake();

    transferSub = new Transfer();

    shooterSub = new Shooter();

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

    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureDriverBindings(){
    /* Driver Controls */

    //driver buttton to zero gyro
    driverController.povUp().onTrue(new InstantCommand(() -> swerveBase.zeroGyro()));

    //bring down arm to stow/intake position then run intake command until note is detected
    driverController.leftBumper().whileTrue((armSub.prepareForIntakeCommand()
            .andThen(new IntakeNoteReg(
              intakeSub, 
              IntakeConstants.intakeSpeed, 
              transferSub, 
              TransferConstants.transferSeed))));
    /*
    driverController.leftBumper().onTrue((armSub.prepareForIntakeCommand()
      .andThen(new IntakeNoteReg(
        intakeSub, 
        IntakeConstants.intakeSpeed, 
        transferSub, 
        TransferConstants.transferSeed))));
    */
  }

  private void configureOperatorBindings(){
  /*add vision to determine speed and angle necessary to score in speaker */

  //manual shooter based on right operator trigger axis value [raw speed]
  operatorController.rightTrigger().whileTrue(
    new InstantCommand(() -> shooterSub.setShooterSpeed(operatorController.getRightTriggerAxis())));

    //Feed Note to shooter [run transfer]
    operatorController.rightBumper().whileTrue(new InstantCommand(() -> transferSub.setTransferSpeed(TransferConstants.transferSeed)));

    //intake from shooter [run shooter in reverse]
    operatorController.leftBumper().whileTrue(
      new InstantCommand(() -> shooterSub.setShooterSpeed(LauncherConstants.intakeFromShooterSpeed)));

    /* Arm related commands */
    //Arm default command [manual arm movment using left stick and left Y joystick axis]
    armSub.setDefaultCommand(new ArmDefault(armSub, operatorController.leftStick(), ()-> -operatorController.getRightY()));

    //stow arm if not already
    operatorController.povRight().onTrue(new prepToShoot(RobotConstants.STOWED, ()-> transferSub.isNoteInTransfer(), armSub, shooterSub));
    
    //bindings to set arm and shooter setpoints
    operatorController.y().onTrue(new prepToShoot(RobotConstants.AMP, ()-> transferSub.isNoteInTransfer(), armSub, shooterSub));
    operatorController.b().onTrue(new prepToShoot(RobotConstants.WING, ()-> transferSub.isNoteInTransfer(),  armSub, shooterSub));
    operatorController.a().onTrue(new prepToShoot(RobotConstants.SPEAKER, ()-> transferSub.isNoteInTransfer(), armSub, shooterSub));
    operatorController.x().onTrue(new prepToShoot(RobotConstants.PODIUM, ()-> transferSub.isNoteInTransfer(), armSub, shooterSub));

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

    //saftey stop for launcher
    operatorController.povLeft().onTrue(new InstantCommand(() -> shooterSub.stopShooter()));


    if (RobotConstants.isShooterTuningMode) {
      SmartDashboard.putData("Update Shooter Gains", shooterSub.updateShooterGainsCommand());
      SmartDashboard.putData("Run Shooter", shooterSub.runShooterCommand());
      SmartDashboard.putData("Stop Shooter", shooterSub.stopShooterCommand());
      SmartDashboard.putData("Arm to Angle", armSub.moveToDegreeCommand());
    }
    SmartDashboard.putData("Move Arm To Setpoint", armSub.tuneArmSetPointCommand());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.plaf.TreeUI;

import org.ejml.equation.Sequence;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.TransferConstants;
//import frc.robot.Util.Setpoints.GameState;
import frc.robot.commands.IntakeNoteReg;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.manualIntakeControl;
import frc.robot.commands.manualTransferControl;
import frc.robot.commands.autocommands.autoCommands;


import frc.robot.subsystems.drive.SwerveBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Shooter;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.transfer.Transfer;

public class RobotContainer {
  //Controllers
  private final CommandXboxController driverController = new CommandXboxController(ControllerConstants.driverControllerPort);
  public final CommandXboxController operatorController = new CommandXboxController(ControllerConstants.operatorControllerPort);

  private final SwerveBase swerveBase;
  private final Intake intakeSub;
  private final Transfer transferSub;
  private final Shooter shooterSub;
  

  private final Leds ledSub;

  private TeleopSwerve teleopSwerve;

  private final SendableChooser<Command> autoChooser;



  public RobotContainer() {

    //initialize subsytems
    swerveBase = new SwerveBase();

    intakeSub = new Intake();

    transferSub = new Transfer();

    shooterSub = new Shooter();

    ledSub = new Leds();



    // Register Named Commands
    NamedCommands.registerCommand("autoIntake", autoCommands.intakeNote(intakeSub, transferSub, ledSub));
    NamedCommands.registerCommand("shootNote", autoCommands.shootNote(shooterSub, transferSub));


    teleopSwerve = new TeleopSwerve(
      swerveBase,
      //inverts controls because joysticks are back-right (+) while robot is front-left (+)
      () -> -driverController.getLeftY(),
      () -> -driverController.getLeftX(),
      () -> -driverController.getRightX(),
      () -> driverController.rightBumper().getAsBoolean(),
      () -> driverController.leftBumper().getAsBoolean()//used to decrease speed of chassis
      );
    
    swerveBase.setDefaultCommand(teleopSwerve);

    configureDriverBindings();
    configureOperatorBindings();

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureDriverBindings(){
    /* Driver Controls */

    //Y Button: Zero Gyro
    driverController.y().onTrue(new InstantCommand(() -> swerveBase.zeroGyro()));

  }

  private void configureOperatorBindings(){
  /*add vision to determine speed and angle necessary to score in speaker */

  //Right Trigger: Manual speaker speed
  operatorController.rightTrigger(.3).whileTrue(
    new InstantCommand(() -> shooterSub.setShooterSpeed(.50))).onFalse(new InstantCommand(()-> shooterSub.stopShooter()));

  //Left Trigger: Manual Amp speed
  operatorController.leftTrigger(.3).whileTrue(
    new InstantCommand(() -> shooterSub.setShooterSpeed(.18))).onFalse(new InstantCommand(()-> shooterSub.stopShooter()));//origin is .20

  //Right Bumber: Feed Note to shooter [run transfer]
  operatorController.rightBumper().whileTrue(new manualTransferControl(transferSub, TransferConstants.transferSeed));



    //Left Bumber: Auto intake *Press once and it will run until canceled or overwritten*
    operatorController.leftBumper().onTrue(
      new IntakeNoteReg(
          intakeSub, IntakeConstants.intakeSpeed,
          transferSub, TransferConstants.transferSeed,
          ledSub));


    //Dpad up: manually intake note
    operatorController.povUp().whileTrue(
      new manualIntakeControl(
        intakeSub, IntakeConstants.intakeSpeed,
        transferSub, TransferConstants.transferSeed));

    //Dpad up: manually spit out
    operatorController.povDown().whileTrue(
      new manualIntakeControl(
        intakeSub, IntakeConstants.intakeSpitSpeed,
        transferSub, TransferConstants.tranSpitSpeed));

    //POV Left: Manual stop for launcher, transfer, intake
    operatorController.povLeft().onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> shooterSub.stopShooter()),
      new InstantCommand(() -> transferSub.stopTransfer()),
      new InstantCommand(() -> intakeSub.stopIntake())));

  }

  //back up auto
  private Command preloadAutoAuton(){
    return new InstantCommand(() -> shooterSub.setShooterSpeed(.5)).andThen(Commands.waitSeconds(7)).andThen(new InstantCommand(() -> transferSub.setTransferSpeed(.35))).andThen(Commands.waitSeconds(2)).andThen(new InstantCommand(() -> shooterSub.stopShooter())).andThen(new InstantCommand(() -> transferSub.stopTransfer()));
  }

/* 
  public void resetModulesTest(){
    swerveBase.resetToAbsolute();
  }
*/


  public Command getAutonomousCommand() {
    //return preloadAutoAuton();
    return autoChooser.getSelected();
}
}
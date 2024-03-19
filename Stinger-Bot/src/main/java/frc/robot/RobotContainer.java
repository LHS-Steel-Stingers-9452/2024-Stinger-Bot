// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ejml.equation.Sequence;

import com.pathplanner.lib.auto.AutoBuilder;
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
  public final CommandXboxController operatorController = new CommandXboxController(ControllerConstants.operatorControllerPort);

  private final SwerveBase swerveBase;
  private final Intake intakeSub;
  private final Transfer transferSub;
  private final Shooter shooterSub;
  private final Arm armSub;

  private TeleopSwerve teleopSwerve;

  private final SendableChooser<Command> autoChooser;



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

    //driver buttton to zero gyro
    driverController.y().onTrue(new InstantCommand(() -> swerveBase.zeroGyro()));

    //bring down arm to stow/intake position then run intake command until note is detected
    /* 
    driverController.leftBumper().onTrue((armSub.prepareForIntakeCommand()
            .andThen(new IntakeNoteReg(
              intakeSub, 
              IntakeConstants.intakeSpeed, 
              transferSub, 
              TransferConstants.transferSeed))));
    */
    
    //Dpad up: manually intake note without photo sensor
    /* 
    driverController.leftBumper().whileTrue(
      new manualIntakeControl(
        intakeSub, IntakeConstants.intakeSpeed,
        transferSub, TransferConstants.transferSeed));
    */
  }

  private void configureOperatorBindings(){
  /*add vision to determine speed and angle necessary to score in speaker */

  //manual shooter control for speaker shot
  operatorController.rightTrigger(.4).whileTrue(
    new InstantCommand(() -> shooterSub.setShooterSpeed(.50))).onFalse(new InstantCommand(()-> shooterSub.stopShooter()));

  operatorController.leftTrigger(.4).whileTrue(
    new InstantCommand(() -> shooterSub.setShooterSpeed(.20))).onFalse(new InstantCommand(()-> shooterSub.stopShooter()));

    //Feed Note to shooter [run transfer]
    operatorController.rightBumper().whileTrue(new manualTransferControl(transferSub, TransferConstants.transferSeed));


    /* Arm related commands */
    //Arm default command [manual arm movment using left stick and left Y joystick axis]
    //armSub.setDefaultCommand(new ArmDefault(armSub, operatorController.leftStick(), ()-> -operatorController.getRightY()));

    //stow arm if not already
    operatorController.povRight().onTrue(new prepToShoot(RobotConstants.STOWED, armSub, shooterSub));
    
    //bindings to set arm and shooter setpoints
    operatorController.a().onTrue(new prepToShoot(RobotConstants.AMP, armSub, shooterSub));
    operatorController.y().onTrue(new prepToShoot(RobotConstants.SPEAKER, armSub, shooterSub));

    //Dpad up: manually intake note without photo sensor
    operatorController.povUp().whileTrue(
      new manualIntakeControl(
        intakeSub, IntakeConstants.intakeSpeed,
        transferSub, TransferConstants.transferSeed));

    //Dpad up: manually outtake note no photo sensor
    operatorController.povDown().whileTrue(
      new manualIntakeControl(
        intakeSub, IntakeConstants.intakeSpitSpeed,
        transferSub, TransferConstants.tranSpitSpeed));

    //auto intake
    operatorController.leftBumper().onTrue(
      new IntakeNoteReg(
        intakeSub, IntakeConstants.intakeSpeed,
        transferSub, TransferConstants.transferSeed));

    //manual stop for launcher and transfer and intake now
    operatorController.povLeft().onTrue(new InstantCommand(() -> shooterSub.stopShooter()));
    operatorController.povLeft().onTrue(new InstantCommand(() -> transferSub.stopTransfer()));
    operatorController.povLeft().onTrue(new InstantCommand(() -> intakeSub.stopIntake()));


    if (RobotConstants.isShooterTuningMode) {
      SmartDashboard.putData("Update Shooter Gains", shooterSub.updateShooterGainsCommand());
      SmartDashboard.putData("Run Shooter", shooterSub.runShooterCommand());
      SmartDashboard.putData("Stop Shooter", shooterSub.stopShooterCommand());
      SmartDashboard.putData("Arm to Angle", armSub.moveToDegreeCommand());
    }
    //SmartDashboard.putData("Move Arm To Setpoint", armSub.tuneArmSetPointCommand());
  }

  private Command preloadAutoAuton(){
    return new InstantCommand(() -> shooterSub.setShooterSpeed(.5)).andThen(Commands.waitSeconds(7)).andThen(new InstantCommand(() -> transferSub.setTransferSpeed(.35))).andThen(Commands.waitSeconds(2)).andThen(new InstantCommand(() -> shooterSub.stopShooter())).andThen(new InstantCommand(() -> transferSub.stopTransfer()));
  }


  public Command getAutonomousCommand() {
    //return preloadAutoAuton();
    return autoChooser.getSelected();
}
}
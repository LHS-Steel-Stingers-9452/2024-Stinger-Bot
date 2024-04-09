// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.CommandManager;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.autoCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.PivotStates;
import frc.robot.subsystems.climbers.Climbers;
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
  private final Arm armSub;
  private final Shooter shooterSub;
  private final Climbers climberSub;
  

  private final Leds ledSub;

  private final SendableChooser<Command> autoChooser;


  public RobotContainer() {

    //initialize subsytems
    swerveBase = new SwerveBase();

    intakeSub = new Intake();

    transferSub = new Transfer();

    armSub = new Arm(null);

    shooterSub = new Shooter();

    ledSub = new Leds();

    climberSub = new Climbers();

    //Try moving where bindings are configured
    //Configures Control Bindings
    configureDriverBindings();
    configureOperatorBindings();

    // Register Named Commands
    NamedCommands.registerCommand("autoIntake", autoCommands.intakeNote(intakeSub, transferSub));
    NamedCommands.registerCommand("shootNote", autoCommands.shootNote(shooterSub, transferSub));
    NamedCommands.registerCommand("midArmShot", autoCommands.midArmShot(armSub, shooterSub, transferSub));

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureDriverBindings(){
  /* Driver Controls */
    
    swerveBase.setDefaultCommand(
      new TeleopSwerve(
        swerveBase,
        //inverts controls because joysticks are back-right (+) while robot is front-left (+)
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> -driverController.getRightX(),
        () -> driverController.rightBumper().getAsBoolean(),//used to drive robot relative
        () -> driverController.leftBumper().getAsBoolean()//used to decrease speed of chassis
        ));

  //Y Button: Zero Gyro
    driverController.y().onTrue(CommandManager.zeroGyro(swerveBase));

  }

  private void configureOperatorBindings(){
  /*add vision to determine speed and angle necessary to score in speaker */

    /**
     * Intake Related Bindings
     * */
    //Left Bumber: Auto intake *Press once and it will run until canceled or overwritten*
    operatorController.leftBumper().onTrue(
      autoCommands.intakeNote(intakeSub, transferSub));

    //Dpad up: manually intake
    operatorController.povUp().whileTrue(
      CommandManager.intakeNote(intakeSub, transferSub)).onFalse(CommandManager.eStop(intakeSub, transferSub));//added eStop() as a saftey thing

    //Dpad down: manually spit out
    operatorController.povDown().whileTrue(
      CommandManager.groundOuttake(intakeSub, transferSub)).onFalse(CommandManager.eStop(intakeSub, transferSub));

    //POV Left: E stop for intake and trasnfer [Added requirements on Instant Commands so should interrupt autoIntaking]
    operatorController.start().onTrue(
      CommandManager.eStop(intakeSub, transferSub));

  /**
  * Arm Related Bindings
  * */
  //test for amp
    operatorController.y()
      .onTrue(
        new InstantCommand(()-> armSub.requestState(PivotStates.AmpState), armSub));

    operatorController.b()
      .onTrue(
        new InstantCommand(()-> armSub.requestState(PivotStates.MidState), armSub));
    
    operatorController.a()
      .onTrue(
        new InstantCommand(()-> armSub.requestState(PivotStates.DefaultState), armSub));

  /**
  * Shoot Related Bindings
  * */
  //Right Trigger: Manual speaker speed [dutycycle]
    operatorController.povRight().whileTrue(
      new InstantCommand(() -> shooterSub.setShooterSpeed(LauncherConstants.dutySpeakerShot))).onFalse(new InstantCommand(()-> shooterSub.stopShooter()));

    operatorController.rightTrigger().whileTrue(
      new InstantCommand(() -> shooterSub.setShooterSpeed(0.60))).onFalse(new InstantCommand(()-> shooterSub.stopShooter()));

    //Trap shot
    operatorController.rightStick().whileTrue(new InstantCommand(() -> shooterSub.setShooterSpeed(0.38))).onFalse(new InstantCommand(()-> shooterSub.stopShooter()));

  //Left Trigger: Manual Amp speed
    operatorController.povLeft().whileTrue(
      new InstantCommand(() -> shooterSub.setShooterSpeed(0.25))).onFalse(new InstantCommand(()-> shooterSub.stopShooter()));//origin is .20

  //Right Bumber: Feed Note to shooter [run transfer]
    operatorController.rightBumper().whileTrue(
      CommandManager.feedNote(transferSub)).onFalse(new InstantCommand(()-> transferSub.stopTransfer()));
  /**
  * Climb Related Bindings
  * */
  //up climbers
  driverController.rightTrigger().whileTrue(new InstantCommand(()-> climberSub.moveup())).whileFalse(new InstantCommand(()-> climberSub.stopClimber()));
  //downClimbers
  driverController.leftTrigger().whileTrue(new InstantCommand(() -> climberSub.moveDown())).whileFalse(new InstantCommand(()-> climberSub.stopClimber()));


  /**
   * Tunning stuff
   */
  if (RobotConstants.isShooterTuningMode) {
      SmartDashboard.putData("Update Shooter Gains", shooterSub.updateShooterGainsCommand());
      SmartDashboard.putData("Run Shooter", shooterSub.runShooterCommand());
      SmartDashboard.putData("Stop Shooter", shooterSub.stopShooterCommand());
    }
  }


  //[OG Saftey Auto]
  private Command preloadAutoAuton(){
    return new InstantCommand(
      () -> shooterSub.setShooterSpeed(.5)).andThen(Commands.waitSeconds(7)).andThen(new InstantCommand(() -> transferSub.setTransferSpeed(.35))).andThen(Commands.waitSeconds(2)).andThen(new InstantCommand(() -> shooterSub.stopShooter())).andThen(new InstantCommand(() -> transferSub.stopTransfer()));
  }


  public Command getAutonomousCommand() {
    //return preloadAutoAuton();
    return autoChooser.getSelected();
}
}
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
import frc.robot.commands.CommandManager;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.autoCommands;
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

    //Try moving where bindings are configured
    //Configures Control Bindings
    configureDriverBindings();
    configureOperatorBindings();

    // Register Named Commands
    NamedCommands.registerCommand("autoIntake", autoCommands.intakeNote(intakeSub, transferSub));
    NamedCommands.registerCommand("shootNote", autoCommands.shootNote(shooterSub, transferSub));

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
    operatorController.povLeft().onTrue(
      CommandManager.eStop(intakeSub, transferSub));

  /**
  * Arm Related Bindings
  * */


  /**
  * Shoot Related Bindings
  * */
  //Right Trigger: Manual speaker speed [dutycycle]
    operatorController.rightTrigger(.3).whileTrue(
      new InstantCommand(() -> shooterSub.setShooterSpeed(.50))).onFalse(new InstantCommand(()-> shooterSub.stopShooter()));

  //Left Trigger: Manual Amp speed
    operatorController.leftTrigger(.3).whileTrue(
      new InstantCommand(() -> shooterSub.setShooterSpeed(.18))).onFalse(new InstantCommand(()-> shooterSub.stopShooter()));//origin is .20

  //Right Bumber: Feed Note to shooter [run transfer]
    operatorController.rightBumper().whileTrue(
      CommandManager.feedNote(transferSub));
  /**
  * Climb Related Bindings
  * */
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
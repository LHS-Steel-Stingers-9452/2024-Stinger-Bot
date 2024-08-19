// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.drive.SwerveBase;

import frc.robot.LimelightHelpers;

public class TeleopSwerve extends Command {
  /** Creates a new TeleopSwerve. */
  private SwerveBase swerveBase;

  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;

  private BooleanSupplier robotCentricSup;
  private BooleanSupplier slowChassisSup;

  private BooleanSupplier lockTagSup;

  public TeleopSwerve( SwerveBase swerveBase,
  DoubleSupplier translationSup,
  DoubleSupplier strafeSup,
  DoubleSupplier rotationSup,
  BooleanSupplier robotCentricSup,
  BooleanSupplier slowChassisSup,
  BooleanSupplier lockTagSup) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveBase = swerveBase;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;    
    this.slowChassisSup = slowChassisSup;
    this.lockTagSup = lockTagSup;

    addRequirements(swerveBase);
  }

  @Override
  public void initialize(){}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double translationVal =
           MathUtil.applyDeadband(translationSup.getAsDouble(), ControllerConstants.deadbandRange);

    double strafeVal =
            MathUtil.applyDeadband(strafeSup.getAsDouble(), ControllerConstants.deadbandRange);

    double rotationVal =
            MathUtil.applyDeadband(rotationSup.getAsDouble(), ControllerConstants.deadbandRange);

    boolean isChassisSlow = 
            slowChassisSup.getAsBoolean();

    boolean isLockTag =
            lockTagSup.getAsBoolean();

    //If left bumper is held slow down chassis to a quarter of 4.6 m/s
    if (isChassisSlow) {
      swerveBase.drive(
      (new Translation2d(translationVal, strafeVal).times(Swerve.maxSpeed).times(0.25)),
      ((rotationVal)*Swerve.maxAngleVelocity),
      (!robotCentricSup.getAsBoolean()),
      (Swerve.openLoopDrive));
    } else if(isLockTag) {
      swerveBase.drive(
      (new Translation2d(translationVal, strafeVal).times(Swerve.maxSpeed).times(0.25)),
      (CommandManager.limelightAim()),// should control robot rotation and lock onto april tags
      (!robotCentricSup.getAsBoolean()),
      (Swerve.openLoopDrive));
    }

    swerveBase.drive(
      (new Translation2d(translationVal, strafeVal).times(Swerve.maxSpeed)),
      (rotationVal)*Swerve.maxAngleVelocity,
      (!robotCentricSup.getAsBoolean()),
      (Swerve.openLoopDrive));
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //set neural output function here to stop motors
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.drive.SwerveBase;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class TeleopSwerve extends Command {
  /** Creates a new TeleopSwerve. */
  private SwerveBase swerveBase;

  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;

  private BooleanSupplier robotCentricSup;

  private SlewRateLimiter translationFilter = new SlewRateLimiter(ControllerConstants.slewRate);
  private SlewRateLimiter strafeFilter = new SlewRateLimiter(ControllerConstants.slewRate);
  private SlewRateLimiter rotationFilter = new SlewRateLimiter(ControllerConstants.slewRate);
  //limiters to soften control inputs

  public TeleopSwerve( SwerveBase swerveBase,
  DoubleSupplier translationSup,
  DoubleSupplier strafeSup,
  DoubleSupplier rotationSup,
  BooleanSupplier robotCentricSup) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveBase = swerveBase;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;    

    addRequirements(swerveBase);
  }

  @Override
  public void initialize(){}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //apply filters to control inputs
    double translationVal =
           translationFilter.calculate(MathUtil.applyDeadband(translationSup.getAsDouble(), ControllerConstants.deadbandRange));

    double strafeVal =
            strafeFilter.calculate(MathUtil.applyDeadband(strafeSup.getAsDouble(), ControllerConstants.deadbandRange));

    double rotationVal =
            rotationFilter.calculate(MathUtil.applyDeadband(rotationSup.getAsDouble(), ControllerConstants.deadbandRange));

    SmartDashboard.putNumber("vX(Teleop)", translationVal);
    SmartDashboard.putNumber("vY(Teleop)", strafeVal);
    SmartDashboard.putNumber("omega(Teleop)", rotationVal);

    swerveBase.drive(
      (new Translation2d(translationVal, strafeVal).times(Swerve.maxSpeed)),
      (rotationVal)*Swerve.maxAngleVelocity,
      (!robotCentricSup.getAsBoolean())
      );

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

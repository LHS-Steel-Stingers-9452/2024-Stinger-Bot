// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.Swerve;
import frc.robot.Constants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.SwerveBase;

public class TeleopSwerve extends Command {
  /** Creates a new TeleopSwerve. */
  private SwerveBase swerveBase;

  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;

  private BooleanSupplier robotCentricSup;

  //limiters to soften control inputs
  private SlewRateLimiter translationFilter = new SlewRateLimiter(ControllerConstants.SLEW_RATE);
  private SlewRateLimiter strafeFilter = new SlewRateLimiter(ControllerConstants.SLEW_RATE);
  private SlewRateLimiter rotationFilter = new SlewRateLimiter(ControllerConstants.SLEW_RATE);

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

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //apply filters to control inputs
    double translationVal = translationFilter.calculate(translationSup.getAsDouble());
    double strafeVal = strafeFilter.calculate(strafeSup.getAsDouble());
    double rotationVal = rotationFilter.calculate(rotationSup.getAsDouble());


    //apply new values to drive function in swerveBase file
    swerveBase.drive(
      (new Translation2d(translationVal, strafeVal).times(Swerve.maxDriveSpeed)),
      (rotationVal * Swerve.maxAngleVelocity),
      (!robotCentricSup.getAsBoolean())//,
      //(Swerve.openLoop)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveBase.drive(new Translation2d(0, 0), 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

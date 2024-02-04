// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
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
    double translationVal =
            MathUtil.applyDeadband(translationSup.getAsDouble(), ControllerConstants.DEADBANDRANGE);

    double strafeVal =
            MathUtil.applyDeadband(strafeSup.getAsDouble(), ControllerConstants.DEADBANDRANGE);

    double rotationVal =
            MathUtil.applyDeadband(rotationSup.getAsDouble(), ControllerConstants.DEADBANDRANGE);


    //apply new values to drive function in swerveBase file
    swerveBase.drive(
      (new Translation2d(translationVal, strafeVal)),
      (rotationVal),
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

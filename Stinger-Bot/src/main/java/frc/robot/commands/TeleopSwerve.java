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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.drive.SwerveBase;

public class TeleopSwerve extends Command {
  /** Creates a new TeleopSwerve. */
  private SwerveBase swerveBase;

  private DoubleSupplier xSupplier;
  private DoubleSupplier ySupplier;
  private DoubleSupplier rotationSup;

  private BooleanSupplier robotCentricSup;
  private BooleanSupplier slowChassisSup;

  private BooleanSupplier lockSpeakerSup;

  private PIDController rotPIDController;

  private GenericEntry rotationError;

  public TeleopSwerve( SwerveBase swerveBase,
  DoubleSupplier xSupplier,
  DoubleSupplier ySupplier,
  DoubleSupplier rotationSup,
  BooleanSupplier robotCentricSup,
  BooleanSupplier slowChassisSup,
  BooleanSupplier lockSpeakerSup) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveBase = swerveBase;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;    
    this.slowChassisSup = slowChassisSup;
    this.lockSpeakerSup = lockSpeakerSup;

    //TODO - Values need to be tuned
    rotPIDController = new PIDController(.008, 0, 0.0010);//.3 origin
    rotPIDController.enableContinuousInput(-180, 180);

    rotationError = Shuffleboard.getTab("vision").add("rot error[deg]", 0).getEntry();

    addRequirements(swerveBase);
  }

  @Override
  public void initialize(){}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xSpeedVal =
           MathUtil.applyDeadband(xSupplier.getAsDouble(), ControllerConstants.deadbandRange);

    double ySpeedVal =
            MathUtil.applyDeadband(ySupplier.getAsDouble(), ControllerConstants.deadbandRange);

    double rotationVal =
            MathUtil.applyDeadband(rotationSup.getAsDouble(), ControllerConstants.deadbandRange);

    boolean isChassisSlow = 
            slowChassisSup.getAsBoolean();

    boolean isLockSpeaker =
            lockSpeakerSup.getAsBoolean();

    if (isChassisSlow){
      xSpeedVal *= 0.25;
      ySpeedVal *= 0.25;
    }

    if(isLockSpeaker){
      //NOTE - Currently getting error in degrees
      rotationVal = 
        //TODO -  test and negate if robot is moving opposite of desired rotation
        -rotPIDController.calculate(180 - swerveBase.getGyroYaw().getDegrees(), swerveBase.rotToSpeaker().getDegrees());
        rotationError.setDouble(180 - rotPIDController.getPositionError());
    }
    
    swerveBase.drive(
      (new Translation2d(xSpeedVal, ySpeedVal).times(Swerve.maxSpeed)),
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
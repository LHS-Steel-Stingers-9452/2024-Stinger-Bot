// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;
//import frc.robot.Constants;
import frc.robot.Constants.Swerve;
//import frc.robot.subsystems.SwerveModule;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SwerveBase extends SubsystemBase {
  /** Creates a new SwerveBase. */
  private final Pigeon2 pidgeotto;

  private final SwerveDriveOdometry swerveOdometry;
  private final SwerveModule[] swerveModules;

  private Field2d field;


  public SwerveBase() {
    pidgeotto = new Pigeon2(Swerve.PIGEON_ID);
    zeroGyro();

    swerveModules = new SwerveModule[] {
      new SwerveModule(0, Swerve.Mod0.constants),
      new SwerveModule(1, Swerve.Mod1.constants),
      new SwerveModule(2, Swerve.Mod2.constants),
      new SwerveModule(3, Swerve.Mod3.constants)
    };

    //Odometry
    swerveOdometry = new SwerveDriveOdometry(Swerve.KINEMATICS, getYaw(), getPositions());

    field = new Field2d();
    SmartDashboard.putData("Field", field);


  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop){

    //Converts joystick inputs to either field relative or chassis speeds using kinematics
    SwerveModuleState [] swerveModuleStates = 
      Swerve.KINEMATICS.toSwerveModuleStates(
        fieldRelative 
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
          //Swapped getY() and getX()
            translation.getY(), translation.getX(), rotation, getYaw())
        : new ChassisSpeeds(translation.getY(), translation.getX(), rotation));

    //Swerve version of normalizing wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Swerve.maxSpeed);

    for (SwerveModule module : swerveModules){
      module.setDesiredState(swerveModuleStates[module.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Swerve.maxSpeed);

    for (SwerveModule mod : swerveModules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }


  public void resetOdometry(Pose2d newPose){
    swerveOdometry.resetPosition(pidgeotto.getRotation2d(), getPositions(), newPose);
  }

  public void zeroGyro(){
    pidgeotto.setYaw(0);
  }

  public Pose2d getPose(){
    return swerveOdometry.getPoseMeters();
  }

  public Rotation2d getYaw(){
    //test
    return (Swerve.invertGyro)
        ? Rotation2d.fromRotations(360 - pidgeotto.getYaw().getValue())
        : Rotation2d.fromRotations(pidgeotto.getYaw().getValue());
  }



  public SwerveModuleState[] getStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];

    for (SwerveModule mod : swerveModules) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }


  public SwerveModulePosition[] getPositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[]{
      new SwerveModulePosition(swerveModules[0].getPosition().distanceMeters, swerveModules[0].getCanCoderValue()),
      new SwerveModulePosition(swerveModules[1].getPosition().distanceMeters, swerveModules[1].getCanCoderValue()),
      new SwerveModulePosition(swerveModules[2].getPosition().distanceMeters, swerveModules[2].getCanCoderValue()),
      new SwerveModulePosition(swerveModules[3].getPosition().distanceMeters, swerveModules[3].getCanCoderValue())
    };
  return positions;
  }





  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    swerveOdometry.update(getYaw(), getPositions());
    field.setRobotPose(getPose());

    //display gyro for fun
    SmartDashboard.putString("Gyro", getYaw().toString());


    for (SwerveModule module : swerveModules) {
      SmartDashboard.putNumber(
          "Mod " + module.moduleNumber + " Cancoder", module.getCanCoderValue().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + module.moduleNumber + " Integrated", module.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + module.moduleNumber + " Velocity", module.getState().speedMetersPerSecond);
    }

    double loggingStates[] = {
      swerveModules[0].getState().angle.getDegrees(), swerveModules[0].getState().speedMetersPerSecond,
      swerveModules[1].getState().angle.getDegrees(), swerveModules[1].getState().speedMetersPerSecond,
      swerveModules[2].getState().angle.getDegrees(), swerveModules[2].getState().speedMetersPerSecond,
      swerveModules[3].getState().angle.getDegrees(), swerveModules[3].getState().speedMetersPerSecond,
    };

    SmartDashboard.putNumberArray("SwerveModule States", loggingStates);
    
  }
}

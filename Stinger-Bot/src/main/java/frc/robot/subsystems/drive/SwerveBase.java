// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve.Mod0;
import frc.robot.Constants.Swerve.Mod1;
import frc.robot.Constants.Swerve.Mod2;
import frc.robot.Constants.Swerve.Mod3;

import static frc.robot.Constants.Swerve.*;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


//Favorite import?
import edu.wpi.first.wpilibj.Timer;


public class SwerveBase extends SubsystemBase {
  /** Creates a new SwerveBase. */
  private final Pigeon2 pidgeotto;

  private final SwerveDriveOdometry swerveOdometry;
  private final SwerveModule[] swerveModules;

  private Field2d field;

  public SwerveBase() {
    pidgeotto = new Pigeon2(pigeonID);
    //zeroGyro();
    pidgeotto.setYaw(0);

    swerveModules = new SwerveModule[] {
      new SwerveModule(0, Mod0.constants),
      new SwerveModule(1, Mod1.constants),
      new SwerveModule(2, Mod2.constants),
      new SwerveModule(3, Mod3.constants)
    };

    //Reset to absoute here please
    /*
    * By pausing init for a second before setting module offsets, we avoid a bug
    * with inverting motors.
    * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
    */
    System.out.println("Waiting for one Second before module offsets...");
    Timer.delay(1.0);
    resetModulesToAbsolute();
    
    //Odometry
    swerveOdometry = new SwerveDriveOdometry(kinematics, getGyroYaw(), getPositions());

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::autoDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(0.0100, 0.0, 0.0), // Translation PID constants
                    //[Tune Translation PID]
                    new PIDConstants(0.0045, 0.0, 0.0), // Rotation PID constants
                    maxSpeed, // Max module speed, in m/s
                    0.372680629034, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

    
  }
  StructArrayPublisher<SwerveModuleState> swerveDisplay = NetworkTableInstance.getDefault()
    .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();

  public void drive(Translation2d translation, double rotation, boolean fieldRelative){

    //Converts joystick inputs to either field relative or chassis speeds using kinematics
    SwerveModuleState [] swerveModuleStates = 
      kinematics.toSwerveModuleStates(
        fieldRelative 
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(), translation.getY(), rotation, getHeading())// if not working replace with getYaw()
        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));

    //Swerve version of normalizing wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);

    for (SwerveModule module : swerveModules){
      module.setDesiredState(swerveModuleStates[module.moduleNumber]);
    }
  }

  public void autoDrive(ChassisSpeeds autoChassisSpeeds){
    drive(
      new Translation2d(
        autoChassisSpeeds.vxMetersPerSecond, 
        autoChassisSpeeds.vyMetersPerSecond), 
      autoChassisSpeeds.omegaRadiansPerSecond, 
      false);//try false, setting true could've been the reason for the drive being reverse on red
    //setModuleStates(autoModuleStates);

  }

  /* Used by SwerveControllerCommand in Auto 
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Swerve.maxSpeed);

    for (SwerveModule mod : swerveModules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber]);
    }
  }
  */
  

  //gets module states
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
    

  public Pose2d getPose(){
    return swerveOdometry.getPoseMeters();
  }

  //used to reset odometry
  public void setPose(Pose2d pose){
  swerveOdometry.resetPosition((getGyroYaw()), getPositions(), pose);
}

public Rotation2d getHeading(){
    return getPose().getRotation();
  }

public void setHeading(Rotation2d heading){
  swerveOdometry.resetPosition(
    getGyroYaw(), 
    getPositions(), 
    new Pose2d(getPose().getTranslation(), heading)
    );
  }

  public void zeroGyro(){
    swerveOdometry.resetPosition(
      getGyroYaw(), 
      getPositions(), 
      new Pose2d(getPose().getTranslation(), new Rotation2d())
      );
  }

  public Rotation2d getGyroYaw(){
    return Rotation2d.fromDegrees(pidgeotto.getYaw().getValue());
  }

  public void resetModulesToAbsolute(){
    for(SwerveModule module : swerveModules){
      module.resetToAbsolute();
    }
  }

/* 
  chassis speed x and y rotation veloc and rotational veloc in rotation and r/s
  hyp of x and y
*/
  /**
   * Gets the current robot-relative velocity (x, y and omega) of the robot
   * @return A ChassisSpeeds object of the current robot-relative velocity
   */
  public ChassisSpeeds getRobotVelocity(){
      return kinematics.toChassisSpeeds(getStates());
    }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    swerveOdometry.update(getGyroYaw(), getPositions());
    field.setRobotPose(getPose());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    //Returns the Robot location of the field

    for (SwerveModule module : swerveModules) {
      SmartDashboard.putNumber(
          "Mod " + module.moduleNumber + " Cancoder RAW ", module.getCanCoderValue().getDegrees());
                SmartDashboard.putNumber(
          "Mod " + module.moduleNumber + " Cancoder OFFSET ", module.getOffsetCanCoderValue().getDegrees());

      //SmartDashboard.putNumber(
        //  "Mod " + module.moduleNumber + " Integrated", module.getState().angle.getDegrees());
      //SmartDashboard.putNumber(
        //  "Mod " + module.moduleNumber + " Velocity", module.getState().speedMetersPerSecond);
    }

  //need to make sure turning counter clockwise, angle on angle motor, CCW+
  //encoders reading positive value, logg data 
  /*
  display gyro to test zero heading
  displays gyro yaw in degrees
  startup value should be 0 because of zeroGyro upon deployment
  CCW+
  0 is facing towards directly towards opponent's alliance station
  */
    SmartDashboard.putNumber("Gyro Angle", getGyroYaw().getDegrees());

    swerveDisplay.set(getStates());
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.Swerve.Mod0;
import frc.robot.Constants.Swerve.Mod1;
import frc.robot.Constants.Swerve.Mod2;
import frc.robot.Constants.Swerve.Mod3;

import static frc.robot.Constants.Swerve.*;

import java.sql.Driver;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


//Favorite import?
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;


public class SwerveBase extends SubsystemBase {
  /** Creates a new SwerveBase. */
  private final Pigeon2 pidgeotto;

  private final SwerveDrivePoseEstimator swervePose;
  private final SwerveModule[] swerveModules;

  private Alliance allianceColor;
  private Pose2d speakerPosition;

  StructArrayPublisher<SwerveModulePosition> swerveDisplay;
  StructArrayPublisher<Pose2d> arrayPublisher;
  
  GenericEntry speakerDistance;
  GenericEntry speakerRotation;


  public SwerveBase() {

    pidgeotto = new Pigeon2(pigeonID);
    pidgeotto.setYaw(0);

    swerveModules = new SwerveModule[] {
      new SwerveModule(0, Mod0.constants),
      new SwerveModule(1, Mod1.constants),
      new SwerveModule(2, Mod2.constants),
      new SwerveModule(3, Mod3.constants)
    };

    /*
    * By pausing init for a second before setting module offsets, we avoid a bug
    * with inverting motors.
    * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
    */
    System.out.println("Waiting for one Second before module offsets...");
    Timer.delay(1.0);
    resetModulesToAbsolute();
    
    //Swerve Pose Estimator
    swervePose = new SwerveDrivePoseEstimator(
      kinematics, 
      getGyroYaw(), 
      getPositions(), 
      new Pose2d());

    AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::autoDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(ppTrnlD, ppTrnlI, ppTrnlD), // Translation PID constants
                    new PIDConstants(ppRotP, ppRotI, ppRotD), // Rotation PID constants
                    maxSpeed, // Max module speed, in m/s
                    driveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
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

    //For visualizing swerve on Advantage Scope
   swerveDisplay = NetworkTableInstance.getDefault()
  .getStructArrayTopic("RealOutputs/ModulePosition/Outputs", SwerveModulePosition.struct).publish();

   arrayPublisher = NetworkTableInstance.getDefault()
  .getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();

    speakerDistance =  Shuffleboard.getTab("vision").add("speaker distance[m]",0).getEntry();
    speakerRotation =  Shuffleboard.getTab("vision").add("rotation to speaker[degrees]",0).getEntry();
    
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, Boolean isOpenLoop){

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
      module.setDesiredState(swerveModuleStates[module.moduleNumber], isOpenLoop);
    }
  }

  public void autoDrive(ChassisSpeeds autoChassisSpeeds){
    drive(
      new Translation2d(
        autoChassisSpeeds.vxMetersPerSecond, 
        autoChassisSpeeds.vyMetersPerSecond), 
        autoChassisSpeeds.omegaRadiansPerSecond, 
        false,
        openLoopDrive);

  }

  /**
   * Gets module states
   * @return module states
   */
  public SwerveModuleState[] getStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];

    for (SwerveModule mod : swerveModules) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  /*
   * All getters here
   */
  public SwerveModulePosition[] getPositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for (SwerveModule mod: swerveModules){
      positions[mod.moduleNumber] = mod.getPosition();
    }

    return positions;
  }

  public double[] getDriveTemp (){

    double [] motorTemps = new double[4];

    for (SwerveModule mod: swerveModules){

     motorTemps[mod.moduleNumber] = mod.getDriveTemp();
    }
    return motorTemps;
  }
  
  public double[] getDriveBusVoltage (){

    double [] BusVoltage = new double[4];

    for (SwerveModule mod: swerveModules){

      BusVoltage[mod.moduleNumber] = mod.getDriveBusVoltage();
    }
    return BusVoltage;
  }

  public double[] getDriveOutputCurrent (){

    double [] OutputCurrent = new double[4];

    for (SwerveModule mod: swerveModules){

      OutputCurrent[mod.moduleNumber] = mod.getDriveOutputCurrent();
    }
    return OutputCurrent;
  }

  public Pose2d getPose(){
    return swervePose.getEstimatedPosition();
  }

  public Rotation2d getHeading(){
    return getPose().getRotation();
  }

  public Rotation2d getGyroYaw(){
    return Rotation2d.fromDegrees(pidgeotto.getYaw().getValue());
  }

  /**
   * Gets the current robot-relative velocity (x, y and omega) of the robot
   * @return A ChassisSpeeds object of the current robot-relative velocity
   */
  public ChassisSpeeds getRobotVelocity(){
    return kinematics.toChassisSpeeds(getStates());
  }

  /**
   * 
   * @return current alliance color
   */
  private Alliance getAllianceColor(){
      if(allianceColor == null){
        if(DriverStation.getAlliance().isPresent()){
          allianceColor = DriverStation.getAlliance().get();
        }
      }
      return allianceColor;
  } 

  private Pose2d getSpeakerPos(){
    if(speakerPosition == null) {
      if(getAllianceColor() != null) {
        speakerPosition = (getAllianceColor() == DriverStation.Alliance.Blue) ? Constants.RobotConstants.blueSpeaker
          : Constants.RobotConstants.redSpeaker;
      }
    }

    return speakerPosition;
  }

  //TODO - Get distance to speaker
  //NOTE - Should utlize same logic red for blue speaker, which speaker is already determined in getSpeakerPos()
  public double getDistanceToSpeaker(Pose2d robotPose, Pose2d speakerPosition){
    double distanceToSpeaker;

    if (speakerPosition == null) return 0.0;

    //x diff
    double xDiff = robotPose.getX() - speakerPosition.getX();
    //y diff
    double yDiff = robotPose.getY() - speakerPosition.getY();

    double xPower = Math.pow(xDiff, 2);
    //pathag
    double yPower = Math.pow(yDiff, 2);

    distanceToSpeaker = Math.sqrt(xPower + yPower);

    return distanceToSpeaker;
  }

  //REVIEW - Rotation to speaker based on two methods below
  //REVIEW - Get rotation to speaker blue
  public double getBlueAngleToSpeaker(){
    Pose2d robotPose = swervePose.getEstimatedPosition();
    Pose2d speakerPosition = RobotConstants.blueSpeaker;

    double xDiff = robotPose.getX() - speakerPosition.getX();
    double yDiff = robotPose.getY() - speakerPosition.getY(); 

    return 180 - Math.toDegrees(Math.atan(yDiff/xDiff));
    
  }
  //REVIEW - Get roation to speaker red
  public double getRedAngleToSpeaker(){
    Pose2d robotPose = swervePose.getEstimatedPosition();
    Pose2d speakerPosition = RobotConstants.redSpeaker;

    double xDiff = robotPose.getX() - speakerPosition.getX();
    double yDiff = robotPose.getY() - speakerPosition.getY(); 

    return Math.toDegrees(Math.atan(yDiff/xDiff));
  }

public double calcAngleToSpeaker(){
  if (getAllianceColor()== DriverStation.Alliance.Blue) {
    return getBlueAngleToSpeaker();
  }
  else{
    return getRedAngleToSpeaker();
  }
  
}


//NOTE -  - WE USE THESE
public Rotation2d rotToSpeaker(){
  return Rotation2d.fromDegrees(calcAngleToSpeaker());
}

public double calcDistanceToSpeaker(){
    if (getSpeakerPos()!= null) {
      return getDistanceToSpeaker(swervePose.getEstimatedPosition(), getSpeakerPos());
      
    } else{ 
      return 999;

    }
}


/*
 * All setters are here 
 */

  public void setPose(Pose2d pose){
    swervePose.resetPosition(getGyroYaw(), getPositions(), pose);
}

public void setHeading(Rotation2d heading){
  swervePose.resetPosition(
    getGyroYaw(), 
    getPositions(), 
    new Pose2d(getPose().getTranslation(), heading));
  }

  public void zeroGyro(){
    swervePose.resetPosition(
      getGyroYaw(), 
      getPositions(), 
      new Pose2d(getPose().getTranslation(), new Rotation2d())
      );
  }

  public void resetModulesToAbsolute(){
    for(SwerveModule module : swerveModules){
      module.resetToAbsolute();
    }
  }
  

  @Override
  public void periodic() {
    //NOTE - All robot rotaion/position units must be in degrees OR Rotaton2d such as in our module and robot angle[With the exception of our Arm.java]
    //NOTE - All robot distance traveled/velocity units must be in meters
    //NOTE - All motor velocity must be measured in RPS[Exception on neo products which can be RPS or RPM]

    // This method will be called once per scheduler run
    swervePose.update(getGyroYaw(), getPositions());

    boolean rejectUpdate = false;
    LimelightHelpers.SetRobotOrientation("limelight", swervePose.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2PoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    if(Math.abs(pidgeotto.getRate()) > 720 || mt2PoseEstimate.tagCount == 0){
      rejectUpdate = true;

    } else if(!rejectUpdate){
      //REVIEW - Might have to increase these Vector values
      swervePose.setVisionMeasurementStdDevs(VecBuilder.fill(.9,.9,9999999));
      swervePose.addVisionMeasurement(
        mt2PoseEstimate.pose, 
        mt2PoseEstimate.timestampSeconds);
    }
    
    //Gyro Yaw and Rate
    SmartDashboard.putNumber("Gyro Rate[Deg/S]", pidgeotto.getRate());
    SmartDashboard.putNumber("Gyro Yaw[Deg]", getGyroYaw().getDegrees());

    //Updates data for module visualization on advantage scope
    swerveDisplay.set(getPositions());
    arrayPublisher.set(new Pose2d[] {getPose()});

    speakerDistance.setDouble(calcDistanceToSpeaker());
    speakerRotation.setDouble(rotToSpeaker().getDegrees());
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve.Mod0;
import frc.robot.Constants.Swerve.Mod1;
import frc.robot.Constants.Swerve.Mod2;
import frc.robot.Constants.Swerve.Mod3;
import frc.robot.subsystems.vision.LimeLight;
import frc.robot.LimelightHelpers;

import static frc.robot.Constants.Swerve.*;

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
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


//Favorite import?
import edu.wpi.first.wpilibj.Timer;


public class SwerveBase extends SubsystemBase {
  /** Creates a new SwerveBase. */
  private final Pigeon2 pidgeotto;

  private final SwerveDrivePoseEstimator swervePose;
  private final SwerveModule[] swerveModules;

  private Field2d field;

  LimeLight StingerCam;


  public SwerveBase(LimeLight camera) {

    pidgeotto = new Pigeon2(pigeonID);
    pidgeotto.setYaw(0);

    StingerCam = camera;

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
      getPose());


    field = new Field2d();
    SmartDashboard.putData("Field", field);

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

    
  }
  //For visualizing swerve on Advantage Scope
  StructArrayPublisher<SwerveModuleState> swerveDisplay = NetworkTableInstance.getDefault()
    .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();

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


/*
 * All setters are here 
 */

  public void setPose(Pose2d pose){
    swervePose.resetPosition(getGyroYaw(), getPositions(), pose);
}

public Rotation2d getHeading(){
    return getPose().getRotation();
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

  public Rotation2d getGyroYaw(){
    return Rotation2d.fromDegrees(pidgeotto.getYaw().getValue());
  }

  public void resetModulesToAbsolute(){
    for(SwerveModule module : swerveModules){
      module.resetToAbsolute();
    }
  }

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
    swervePose.update(getGyroYaw(), getPositions());

    StingerCam.setRobotOrientation(swervePose.getEstimatedPosition().getRotation().getDegrees());

    if(!StingerCam.rejectPoseEstimate(pidgeotto.getRate() > 600))
      {
        swervePose.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));//tune these numbers 
        swervePose.addVisionMeasurement(
        StingerCam.getPoseEstimate().pose,
        StingerCam.getPoseEstimate().timestampSeconds);
      }

    field.setRobotPose(getPose());

    //Returns the Robot location of the field
    SmartDashboard.putString("Robot Location coordinates", getPose().getTranslation().toString());
    SmartDashboard.putNumber("Gyro Rate[Deg/S]", pidgeotto.getRate());


    //gyro angle and swerve states and rate
    SmartDashboard.putNumber("Gyro Angle", getGyroYaw().getDegrees());
    swerveDisplay.set(getStates());
  }
}

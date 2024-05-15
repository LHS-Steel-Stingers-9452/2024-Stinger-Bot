// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.LauncherConstants.*;
import frc.robot.Constants.RobotConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StaticBrake;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Shooter extends SubsystemBase {

  /** Creates a new Launcher. */
  private final TalonFX topLauncher = new TalonFX(topLaunchID);
  private final TalonFX bottomLauncher = new TalonFX(bottomLaunchID);


  private TalonFXConfiguration motorConfig = new TalonFXConfiguration();


  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0);
  private double globalTargetVelocity;

  GenericEntry shooterVeloc;
  GenericEntry shooterVelocRPM;
  GenericEntry canShoot;

  public Shooter() {

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.Voltage.PeakForwardVoltage = 12.0;
    motorConfig.Voltage.PeakReverseVoltage = 0.0;

    motorConfig.Slot0.kP = kP;
    motorConfig.Slot0.kI = kI;
    motorConfig.Slot0.kD = kD;
    motorConfig.Slot0.kV = kV;

    /* Apply configs */
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    topLauncher.getConfigurator().apply(motorConfig);
    
    bottomLauncher.setControl(new Follower(topLaunchID, false));

    // optimize StatusSignal rates for the Talons
    topLauncher.getVelocity().setUpdateFrequency(50);
    topLauncher.optimizeBusUtilization();
    bottomLauncher.getVelocity().setUpdateFrequency(50);
    bottomLauncher.optimizeBusUtilization();  

    

    shooterVeloc = Shuffleboard.getTab("Shooter").add("Shooter Velocity[RPS]", 0).getEntry();
    canShoot = Shuffleboard.getTab("Shooter").add("Shooter at speed?", false).getEntry();
    shooterVelocRPM = Shuffleboard.getTab("Shooter").add("Shooter Veloc [RPM]",0).getEntry();

    globalTargetVelocity = 0;
  }

  /**
   * Backup shot using duty cycle 
   * @param speed
   */
  public void setShooterSpeed(double speed){
    topLauncher.set(speed);
    bottomLauncher.set(speed);
  }

  /**
  * @param targetVelocity The target velocity in RPS of the shooter 
  */
  public void runShooter(double targetVelocity) {
    // Save Velocity setpoints
    globalTargetVelocity = targetVelocity;
    topLauncher.setControl(velocityVoltageRequest.withVelocity(targetVelocity));
    bottomLauncher.setControl(velocityVoltageRequest.withVelocity(targetVelocity));
  }

  public void stopShooter(){
    topLauncher.setControl(new StaticBrake());
    bottomLauncher.setControl(new StaticBrake());
  }

  /**
  *
  * @return the velocity of the shooter in RPS
  */
  public double getShooterVelocityRPS() {
    return topLauncher.getVelocity().getValueAsDouble();
  }

  public double getShooterVelocityRPM(){
    return getShooterVelocityRPS() * 60;
  }

    /**
     * @return true if the error of the shooter is within the tolerance
     */
    public boolean areWheelsAtSpeed() {
      double launchError = Math.abs(globalTargetVelocity - getShooterVelocityRPM());
      return launchError < shooterTolerence;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    canShoot.setBoolean(areWheelsAtSpeed());
    shooterVeloc.setDouble(getShooterVelocityRPS());
    shooterVeloc.setDouble(getShooterVelocityRPM());
  }
}

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
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StaticBrake;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;



public class Shooter extends SubsystemBase {

  /** Creates a new Launcher. */
  private final TalonFX topLauncher;
  private final TalonFX bottomLauncher;


  private TalonFXConfiguration motorConfig = new TalonFXConfiguration();

  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0);

  private double targetVelocValue;

  GenericEntry velocRawRPS;
  GenericEntry velocAbsRPS;
  GenericEntry canShoot;

  public Shooter() {

    topLauncher = new TalonFX(topLaunchID);
    bottomLauncher = new TalonFX(bottomLaunchID);

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfig.Voltage.PeakForwardVoltage = 12.0;
    motorConfig.Voltage.PeakReverseVoltage = 12.0;

    /* Update Shooter Gains from TunableNumbers */
    motorConfig.Slot0.kP = kP;
    motorConfig.Slot0.kI = kI;
    motorConfig.Slot0.kD = kD;
    motorConfig.Slot0.kV = kV;

    /* Apply configs */
    //Both motors are CC+
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    topLauncher.getConfigurator().apply(motorConfig);
    
    bottomLauncher.setControl(new Follower(topLaunchID, false));

    // optimize StatusSignal rates for the Talons
    topLauncher.getVelocity().setUpdateFrequency(50);
    topLauncher.optimizeBusUtilization();
    bottomLauncher.getVelocity().setUpdateFrequency(50);
    bottomLauncher.optimizeBusUtilization();  

    velocRawRPS = Shuffleboard.getTab("Shooter").add("Veloc [Raw-RPS]",0).getEntry();
    velocAbsRPS = Shuffleboard.getTab("Shooter").add("Veloc [Abs-RPS]",0).getEntry();
    canShoot = Shuffleboard.getTab("Shooter").add("At speed?", false).getEntry();
  }

  /**
  *
  * @return the velocity of the shooter in RPS
  */
  public double getShooterVelocity() {
    return topLauncher.getVelocity().getValueAsDouble();
  }

    /**
     * @return true if the error of the shooter is within the tolerance
     */
    public boolean areWheelsAtSpeed() {
      double launchError = Math.abs(targetVelocValue - getShooterVelocity());
      return launchError < shooterTolerence;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    velocRawRPS.setDouble(getShooterVelocity());
    velocAbsRPS.setDouble(Math.abs(getShooterVelocity()));

    canShoot.setBoolean(areWheelsAtSpeed());
    if (areWheelsAtSpeed() && (targetVelocValue == 0)){
        coastMode();
    }
  }

    /**
   * Shoot using duty cycle 
   * @param speed value range: [-1,1]
   */
  public void dutyShot(double speed){
    topLauncher.set(speed);
    bottomLauncher.set(speed);
  }

  /**
  * @param targetVelocity The target velocity in RPS 
  */
  public void runShooter(double targetVelocity) {
    targetVelocValue = targetVelocity;
    topLauncher.setControl(velocityVoltageRequest.withVelocity(targetVelocity));
  }

  /**
   * Stop motors with break mode
   */
  public void instantStop(){
    targetVelocValue = 0;
    topLauncher.setControl(new StaticBrake());
  }

  /**
   * Stop motors with coast mode
   */
  private void coastMode(){
    targetVelocValue = 0;
    topLauncher.setControl(new CoastOut());
  }
}

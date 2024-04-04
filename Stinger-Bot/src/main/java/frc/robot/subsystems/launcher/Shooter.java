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

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StaticBrake;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import frc.robot.Util.TunableNumber;



public class Shooter extends SubsystemBase {

  /** Creates a new Launcher. */
  private final TalonFX topLauncher = new TalonFX(topLaunchID);
  private final TalonFX bottomLauncher = new TalonFX(bottomLaunchID);

  private static TunableNumber shooterKP = new TunableNumber("Shooter KP", 0.05);
  private static TunableNumber shooterKI = new TunableNumber("Shooter KI", 0);
  private static TunableNumber shooterKD = new TunableNumber("Shooter KD", 0);
  private static TunableNumber shooterKV = new TunableNumber("Shooter KV", 0.113);

  //Setpoint in RPS
  private static TunableNumber shooterSetPointVal = new TunableNumber("L shooter setpoint", 0);


  private TalonFXConfiguration motorConfig = new TalonFXConfiguration();


  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0);

  GenericEntry shooterVeloc;
  GenericEntry canShoot;

  public Shooter() {

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfig.Voltage.PeakForwardVoltage = 12.0;
    motorConfig.Voltage.PeakReverseVoltage = 0.0;


    /* Update Shooter Gains from TunableNumbers */
    motorConfig.Slot0.kP = shooterKP.get();
    motorConfig.Slot0.kI = shooterKI.get();
    motorConfig.Slot0.kD = shooterKD.get();
    motorConfig.Slot0.kV = shooterKV.get();

    /* Apply configs */
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    topLauncher.getConfigurator().apply(motorConfig);

    // optimize StatusSignal rates for the Talons
    topLauncher.getVelocity().setUpdateFrequency(50);
    topLauncher.optimizeBusUtilization();
    bottomLauncher.getVelocity().setUpdateFrequency(50);
    bottomLauncher.optimizeBusUtilization();  

    

    shooterVeloc = Shuffleboard.getTab("Shooter").add("Shooter Velocity[RPS]", 0).getEntry();
    canShoot = Shuffleboard.getTab("Shooter").add("Shooter at speed?", false).getEntry();
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
    shooterSetPointVal.set(targetVelocity);
    topLauncher.setControl(velocityVoltageRequest.withVelocity(targetVelocity));
  }

  public void runShooter() {
    // Get the velocity setpoint from TunableNumber
    topLauncher.setControl(velocityVoltageRequest.withVelocity(shooterSetPointVal.get()));
  }

  public void stopShooter(){
    topLauncher.setControl(new StaticBrake());
    bottomLauncher.setControl(new StaticBrake());
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
      double launchError = Math.abs(shooterSetPointVal.get() - getShooterVelocity());
      return launchError < shooterTolerence;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    bottomLauncher.setControl(new Follower(topLaunchID, false));

    canShoot.setBoolean(areWheelsAtSpeed());
    shooterVeloc.setDouble(getShooterVelocity());
  }

  /**
  * Update Shooter Gains from TunableNumbers
  */
  public void updateGains() {
    var slot0 = new Slot0Configs();

    slot0.kP = shooterKP.get();
    slot0.kI = shooterKI.get();
    slot0.kD = shooterKD.get();
    slot0.kV = shooterKV.get();

    topLauncher.getConfigurator().apply(slot0);
    //bottomLauncher.getConfigurator().apply(slot0);//just in case
  }

  /**
  * @param shooterVeloc - Requested shooter veloc in RPS
  */
  public void setShooterSetpoints(Double shooterVeloc) {
    shooterSetPointVal.set(shooterVeloc);
  }

  /*
  * Command Factories
  */
  public Command runShooterCommand(double velocity) {
    return new RunCommand(()->this.runShooter(velocity), this);
  }

  public Command runShooterCommand() {
      return new RunCommand(()->this.runShooter(), this);
  }

  public Command stopShooterCommand() {
      return new RunCommand(()->this.stopShooter(), this);
  }


  public Command updateShooterGainsCommand() {
      return new InstantCommand(()->this.updateGains(), this);
  }
  

}

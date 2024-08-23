// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.shooterConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StaticBrake;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;



public class Shooter extends SubsystemBase {

  /** Creates a new Launcher. */
  private final TalonFX topFlywheel;
  private final TalonFX botttomFlywheel;


  private TalonFXConfiguration motorConfig = new TalonFXConfiguration();

  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0);

  private double targetVelocValue;

  GenericEntry velocRawRPS;
  GenericEntry velocAbsRPS;
  GenericEntry canShoot;

  public Shooter() {

    topFlywheel = new TalonFX(topID);
    botttomFlywheel = new TalonFX(bottomID);

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
    topFlywheel.getConfigurator().apply(motorConfig);
    
    botttomFlywheel.setControl(new Follower(topID, false));

    // optimize StatusSignal rates for the Talons
    topFlywheel.getVelocity().setUpdateFrequency(50);
    topFlywheel.optimizeBusUtilization();
    botttomFlywheel.getVelocity().setUpdateFrequency(50);
    botttomFlywheel.optimizeBusUtilization();  

    velocRawRPS = Shuffleboard.getTab("Shooter").add("Veloc [Raw-RPS]",0).getEntry();
    velocAbsRPS = Shuffleboard.getTab("Shooter").add("Veloc [Abs-RPS]",0).getEntry();
    canShoot = Shuffleboard.getTab("Shooter").add("At speed?", false).getEntry();
  }

  /**
  *
  * @return the velocity of the shooter in RPS
  */
  public double getShooterVelocity() {
    return topFlywheel.getVelocity().getValueAsDouble();
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
    topFlywheel.set(speed);
    botttomFlywheel.set(speed);
  }
  
  /**
   * To be used with {@code dutyShot()}
   */
  public void dutyStop(){
    topFlywheel.setControl(new StaticBrake());
    botttomFlywheel.setControl(new StaticBrake());
  }

  /**
  * @param targetVelocity The target velocity in RPS 
  */
  public void runShooter(double targetVelocity) {
    targetVelocValue = targetVelocity;
    topFlywheel.setControl(velocityVoltageRequest.withVelocity(targetVelocity));
  }

  /**
   * Stop motors with break mode
   */
  public void instantStop(){
    targetVelocValue = 0;
    topFlywheel.setControl(new StaticBrake());
  }

  /**
   * Stop motors with coast mode
   */
  private void coastMode(){
    targetVelocValue = 0;
    topFlywheel.setControl(new CoastOut());
  }
}

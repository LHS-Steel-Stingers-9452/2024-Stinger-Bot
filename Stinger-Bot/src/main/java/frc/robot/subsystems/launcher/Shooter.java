// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;



public class Shooter extends SubsystemBase {

  /** Creates a new Launcher. */
  private final TalonFX topFlywheel;
  private final TalonFX botttomFlywheel;

  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0);

  private TalonFXConfiguration motorConfig = new TalonFXConfiguration();

  private double targetVelocValue;

  GenericEntry velocRawRPS;
  GenericEntry velocAbsRPS;
  GenericEntry canShoot;

  //REVIEW - If arm follower works then utilize here as well
  public Shooter() {

    topFlywheel = new TalonFX(topID);
    botttomFlywheel = new TalonFX(bottomID);

    velocRawRPS = Shuffleboard.getTab("Shooter").add("Veloc [Raw-RPS]",0).getEntry();
    velocAbsRPS = Shuffleboard.getTab("Shooter").add("Veloc [Abs-RPS]",0).getEntry();
    canShoot = Shuffleboard.getTab("Shooter").add("At speed?", false).getEntry();

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.Voltage.PeakForwardVoltage = 12.0;
    motorConfig.Voltage.PeakReverseVoltage = -12.0;

    //TODO - Values must be manually tuned
    motorConfig.Slot0.kP = kP;
    motorConfig.Slot0.kI = kI;
    motorConfig.Slot0.kD = kD;
    motorConfig.Slot0.kV = kV;

    /* Apply configs */
    //Both motors are CC+
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    topFlywheel.getConfigurator().apply(motorConfig);
    botttomFlywheel.getConfigurator().apply(motorConfig);
    
    botttomFlywheel.setControl(new Follower(topID, false));

    // optimize StatusSignal rates for the Talons
    topFlywheel.getVelocity().setUpdateFrequency(50);
    topFlywheel.optimizeBusUtilization();
    botttomFlywheel.getVelocity().setUpdateFrequency(50);
    botttomFlywheel.optimizeBusUtilization();  
  }

  /**
  *
  * @return the velocity of the shooter in RPS
  */
  public double getShooterVelocity() {
    return topFlywheel.getVelocity().getValueAsDouble();
  }

    /**
     * @return true if the veloc of the shooter is within the tolerance
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

    //FIXME - Logic could be flaw not allowing the motors to move when requested
    //NOTE - By the time motors leads are shorted coast mode will be applied so the else{} logic is uncessecarry
    //REVIEW - Logic needs secand hand confirmation review, but issues hould be resolved
    /* 
    if ((areWheelsAtSpeed()) && (targetVelocValue == 0)){
      coastMode();
    } 
    */
  }

    /**
   * Shoot using duty cycle 
   * @param speed value range: [-1,1]
   */
  public void dutyShot(double speed){
    topFlywheel.set(speed);
    botttomFlywheel.set(speed);
  }

      public void redirect(double speed){
    topFlywheel.set(speed);
  
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
    topFlywheel.setControl(new NeutralOut());
  }

  /**
   * Stop motors with coast mode
   */
  private void coastMode(){
    targetVelocValue = 0;
    topFlywheel.setControl(new CoastOut());
  }

  public void dutyStop(){

    topFlywheel.setControl(new StaticBrake());
    botttomFlywheel.setControl(new StaticBrake());
  }
}
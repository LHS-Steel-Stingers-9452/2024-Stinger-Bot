// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Util.Setpoints;
import frc.robot.Util.TunableNumber;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Shooter extends SubsystemBase {
  /** Creates a new Launcher. */
  private final TalonFX leftMotor = new TalonFX(LauncherConstants.leftMotorID);
  private final TalonFX rightMotor = new TalonFX(LauncherConstants.rightMotorID);

  private static TunableNumber shooterKP = new TunableNumber("Shooter KP", 0.05);
  private static TunableNumber shooterKI = new TunableNumber("Shooter KI", 0);
  private static TunableNumber shooterKD = new TunableNumber("Shooter KD", 0);
  private static TunableNumber shooterKV = new TunableNumber("Shooter KV", 0.113);

  //Setpoint in RPS
  private static TunableNumber leftShooterSetpointVal = new TunableNumber("L shooter setpoint", 0);
  private static TunableNumber rightShooterSetpointVal = new TunableNumber("R shooter setpoint", 0);


  private TalonFXConfiguration motorConfig = new TalonFXConfiguration();


  private final VelocityVoltage velocityVoltageL = new VelocityVoltage(0);
  private final VelocityVoltage velocityVoltageR = new VelocityVoltage(0);

  public enum kShooterSide {
    kLEFT,
    kRIGHT
  };

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
    leftMotor.getConfigurator().apply(motorConfig);
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightMotor.getConfigurator().apply(motorConfig);

    // optimize StatusSignal rates for the Talons
    leftMotor.getVelocity().setUpdateFrequency(50);
    leftMotor.optimizeBusUtilization();
    rightMotor.getVelocity().setUpdateFrequency(50);
    rightMotor.optimizeBusUtilization();  
  }

  //for manual control
  public void setShooterSpeed(double speed){
    leftMotor.set(speed);
    rightMotor.set(speed);
  }

  /**
  * @param targetVelocity the velocity in RPS of the shooter Right
  * @param targetVelocityL the velocity in RPS of the shooter Left
  */
  public void runShooter(double targetVelocityL, double targetVelocityR) {
    // Save Velocity setpoints
    leftShooterSetpointVal.set(targetVelocityL);
    rightShooterSetpointVal.set(targetVelocityR);
    leftMotor.setControl(velocityVoltageL.withVelocity(targetVelocityL));
    rightMotor.setControl(velocityVoltageR.withVelocity(targetVelocityR));
  }

  public void runShooter() {
    // Get Velocity setpoint from TunableNumber
    leftMotor.setControl(velocityVoltageL.withVelocity(leftShooterSetpointVal.get()));
    rightMotor.setControl(velocityVoltageR.withVelocity(rightShooterSetpointVal.get()));
  }

  public void stopShooter(){
    leftMotor.setControl(new NeutralOut());
    rightMotor.setControl(new NeutralOut());
  }

  /**
  * @param int side - the side of the shooter to query (0 = left, 1 = right)
  * @return the velocity of the specified shooter side in RPS
  */
  public double getShooterVelocity(kShooterSide side) {
    switch(side) {
    case kLEFT:
      return leftMotor.getVelocity().getValueAsDouble();
    case kRIGHT:
      return rightMotor.getVelocity().getValueAsDouble();
    default:
        return 0.0;
    }
  }

    /**
     * @return true if the error of the shooter is within the tolerance
     */
    public boolean areWheelsAtSpeed() {
      double leftErr = Math.abs(leftShooterSetpointVal.get() - getShooterVelocity(kShooterSide.kLEFT));
      double rightErr = Math.abs(rightShooterSetpointVal.get() - getShooterVelocity(kShooterSide.kRIGHT));
      return (leftErr + rightErr / 2.0) < LauncherConstants.shooterTolerence;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Shooter at Speed?", areWheelsAtSpeed());

    if (RobotConstants.isShooterTuningMode) {
      // Put actual velocities to smart dashboard
      SmartDashboard.putNumber("Shooter Velocity L", getShooterVelocity(kShooterSide.kLEFT));
      SmartDashboard.putNumber("Shooter Velocity R", getShooterVelocity(kShooterSide.kRIGHT));
    }
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

    leftMotor.getConfigurator().apply(slot0);
    rightMotor.getConfigurator().apply(slot0);
  }

  /**
  * @param setpoints - Reference to a Setpoints class instance
  */
  public void setShooterSetpoints(Setpoints setpoints) {
    leftShooterSetpointVal.set(setpoints.leftShooter);
    rightShooterSetpointVal.set(setpoints.rightShooter);
  }

  /*
  * Command Factories
  */
  public Command runShooterCommand(double velocityL, double velocityR) {
    return new RunCommand(()->this.runShooter(velocityL, velocityR), this);
  }

  public Command runShooterCommand() {
      return new RunCommand(()->this.runShooter(), this);
  }

    public Command stopShooterCommand() {
      return new InstantCommand(()->this.stopShooter(), this);
  }

  public Command updateShooterGainsCommand() {
      return new InstantCommand(()->this.updateGains(), this);
  }
  

}

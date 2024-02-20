// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Shooter extends SubsystemBase {
  /** Creates a new Launcher. */
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  private TalonFXConfigurator leftMotorConfigurator;
  private TalonFXConfigurator rightMotorConfigurator;

  private TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
  private TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();


  private VelocityVoltage velocityRequest;

  public Shooter() {
    leftMotor = new TalonFX(LauncherConstants.leftMotorID);
    rightMotor = new TalonFX(LauncherConstants.rightMotorID);

    velocityRequest = new VelocityVoltage(0).withSlot(0);

    configLeft();
    configRight(); 
  }

  public void setShooterSpeed(double speed, double feedForwardVal){
    //leftMotor.set(speed);
    leftMotor.setControl(velocityRequest.withVelocity(speed).withFeedForward(feedForwardVal));
    rightMotor.setControl(velocityRequest.withVelocity(speed).withFeedForward(feedForwardVal));
  }

  public void stopShooter(){
    leftMotor.setControl(new NeutralOut());
    rightMotor.setControl(new NeutralOut());
  }

  public double getLeftMotorSpeed(){
    return (leftMotor.getVelocity().getValueAsDouble());
  }

  public double getRightMotorSpeed(){
    return (rightMotor.getVelocity().getValueAsDouble());

  }

  public boolean isLeftMotorAtSpeed(double desiredVelocity, double tolerance){
    return Math.abs(getLeftMotorSpeed() - desiredVelocity) <= tolerance;

  }

  public boolean isRightMotorAtSpeed(double desiredVelocity, double tolerance){
    return Math.abs(getRightMotorSpeed() - desiredVelocity) <= tolerance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("left launcher speed(RPS)", getLeftMotorSpeed());
    SmartDashboard.putNumber("right Launcher speed(RPS)", getRightMotorSpeed());
    
  }

  public void configLeft(){
    leftMotorConfigurator = leftMotor.getConfigurator();

    leftMotorConfigurator.apply(leftMotorConfig);
    //restore factory defaults

    leftMotor.setInverted(false);
    leftMotorConfig.Slot0.kP = LauncherConstants.launcherP;
    leftMotorConfig.Slot0.kI = LauncherConstants.launcherI;
    leftMotorConfig.Slot0.kD = LauncherConstants.launcherD;
    leftMotorConfig.Slot0.kS = LauncherConstants.launchS;
    leftMotorConfig.Slot0.kV = LauncherConstants.launchV;

    leftMotorConfigurator.apply(leftMotorConfig);
  }

  public void configRight(){
    rightMotorConfigurator = rightMotor.getConfigurator();
    rightMotorConfigurator.apply(rightMotorConfig);
    //restore factory defaults

    rightMotor.setInverted(false);
    rightMotorConfig.Slot0.kP = LauncherConstants.launcherP;
    rightMotorConfig.Slot0.kI = LauncherConstants.launcherI;
    rightMotorConfig.Slot0.kD = LauncherConstants.launcherD;
    rightMotorConfig.Slot0.kS = LauncherConstants.launchS;
    rightMotorConfig.Slot0.kV = LauncherConstants.launchV;

    rightMotorConfigurator.apply(rightMotorConfig);
  }
}

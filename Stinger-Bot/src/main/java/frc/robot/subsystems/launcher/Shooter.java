// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Launcher;

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

  private TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
  private TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();

  private TalonFXConfigurator leftMotorConfigurator;
  private TalonFXConfigurator rightMotorConfigurator;

  private VelocityVoltage velocityRequest;

  public Shooter() {
    leftMotor = new TalonFX(Launcher.leftMotorID);
    rightMotor = new TalonFX(Launcher.rightMotorID);

    velocityRequest = new VelocityVoltage(0).withSlot(0);

    configLeft();
    configRight(); 
  }

  public void setShooterSpeed(double speed, double feedForwardVal){
    //leadMotor.set(speed);
    leftMotor.setControl(velocityRequest.withVelocity(speed).withFeedForward(feedForwardVal));
    rightMotor.setControl(velocityRequest.withVelocity(speed).withFeedForward(feedForwardVal));
  }

  public void stopShooter(){
    leftMotor.setControl(new NeutralOut());
    rightMotor.setControl(new NeutralOut());
  }

  public double getLeftMotorSpeed(){
    return leftMotor.getVelocity().getValueAsDouble();
  }

  public boolean isLeftMotorAtSpeed(double desiredVelocity, double tolerance){
    return Math.abs(getLeftMotorSpeed() - desiredVelocity) <= tolerance;

  }

  public double getRightMotorSpeed(){
    return rightMotor.getVelocity().getValueAsDouble();

  }

  public boolean isRightMotorAtSpeed(double desiredVelocity, double tolerance){
    return Math.abs(getRightMotorSpeed() - desiredVelocity) <= tolerance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("lead launcher speed(RPS)", getLeftMotorSpeed());
    SmartDashboard.putNumber("follow Launcher speed(RPS)", getRightMotorSpeed());
    
  }

  public void configLeft(){
    leftMotorConfigurator = leftMotor.getConfigurator();

    leftMotorConfigurator.apply(leftMotorConfig);
    //restore factory defaults

    leftMotor.setInverted(false);
    leftMotorConfig.Slot0.kP = Launcher.launcherP;
    leftMotorConfig.Slot0.kI = Launcher.launcherI;
    leftMotorConfig.Slot0.kD = Launcher.launcherD;
    leftMotorConfig.Slot0.kS = Launcher.launchS;
    leftMotorConfig.Slot0.kV = Launcher.launchV;

    leftMotorConfigurator.apply(leftMotorConfig);
  }

  public void configRight(){
    rightMotorConfigurator = rightMotor.getConfigurator();
    rightMotorConfigurator.apply(rightMotorConfig);
    //restore factory defaults

    rightMotor.setInverted(false);
    rightMotorConfig.Slot0.kP = Launcher.launcherP;
    rightMotorConfig.Slot0.kI = Launcher.launcherI;
    rightMotorConfig.Slot0.kD = Launcher.launcherD;
    rightMotorConfig.Slot0.kS = Launcher.launchS;
    rightMotorConfig.Slot0.kV = Launcher.launchV;

    rightMotorConfigurator.apply(rightMotorConfig);
  }
}

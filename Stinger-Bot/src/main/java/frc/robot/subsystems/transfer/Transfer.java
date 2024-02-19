// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.transfer;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.TransferConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Transfer extends SubsystemBase {
  /** Creates a new Launcher. */

  private final TalonFX transferMotor;

  private TalonFXConfigurator transferConfigurator;
  private TalonFXConfiguration transferConfig = new TalonFXConfiguration();
  

  private VelocityVoltage velocityRequest;

  private CurrentLimitsConfigs transferLimitCurrentConfigs;

  public Transfer() {
    transferMotor = new TalonFX(TransferConstants.transferID);

    transferLimitCurrentConfigs = new CurrentLimitsConfigs();

    velocityRequest = new VelocityVoltage(0).withSlot(0);
    transferConfig();
  }

  public void setTransferSpeed(double speedRPS){
    transferMotor.setControl(velocityRequest.withVelocity(speedRPS).withFeedForward(LauncherConstants.ffOvercomeGrav));
  }

  public void stopTransfer(){
    transferMotor.setControl(new NeutralOut());
  }

  public double getTransferSpeed(){
    return transferMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.getNumber("transfer Speed(RPM)", (getTransferSpeed() * 60));
    SmartDashboard.getNumber("transfer current", transferMotor.getStatorCurrent().getValueAsDouble());
  }

  public void transferConfig(){
    transferConfigurator.apply(transferConfig);
    //restore factory defaults

    transferMotor.setInverted(true);

    transferConfig.Slot0.kP = LauncherConstants.launcherP;
    transferConfig.Slot0.kI = LauncherConstants.launcherI;
    transferConfig.Slot0.kD = LauncherConstants.launcherD;
    transferConfigurator.apply(transferConfig);

    transferLimitCurrentConfigs.withStatorCurrentLimit(TransferConstants.transferCurrentLimit);
    transferLimitCurrentConfigs.withStatorCurrentLimitEnable(true);
    transferConfigurator.apply(transferLimitCurrentConfigs);

    
  }
}

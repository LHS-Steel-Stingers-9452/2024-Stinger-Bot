// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.transfer;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransferConstants;

import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
//import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.NeutralOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Transfer extends SubsystemBase {
  /** Creates a new Launcher. */

  private final TalonFX transferMotor;

  private TalonFXConfigurator transferConfigurator;
  private TalonFXConfiguration transferConfig = new TalonFXConfiguration();

  //private CurrentLimitsConfigs transferLimitCurrentConfigs;

  public Transfer() {
    transferMotor = new TalonFX(TransferConstants.transferID);

    //transferLimitCurrentConfigs = new CurrentLimitsConfigs();

    transferConfig();
  }

  public void setTransferSpeed(double transferSpeed){
    transferMotor.set(transferSpeed);
  }

  public void stopTransfer(){
    transferMotor.setControl(new NeutralOut());
  }

  public double getTransferSpeed(){
    return transferMotor.getVelocity().getValueAsDouble();
  }

  //Use currentLimiting while waiting on photo sensor
  public boolean isNoteInTransfer(){
    double current = transferMotor.getStatorCurrent().getValueAsDouble();
    double currentLimit = TransferConstants.transferCurrentLimit;
    double currentSpeed = getTransferSpeed();
    double speedTolerance = TransferConstants.speedTolerance;

    if (current >= currentLimit && Math.abs(currentSpeed) <= speedTolerance) {
      return true;
    } else {
        return false;
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("transfer Speed(RPS)", getTransferSpeed());
    SmartDashboard.putNumber("transfer Speed(RPM)", getTransferSpeed() * 60);
    SmartDashboard.putNumber("transfer current", transferMotor.getStatorCurrent().getValueAsDouble());
  }

  public void transferConfig(){
    transferConfigurator = transferMotor.getConfigurator();
    transferConfigurator.apply(transferConfig);
    
    //restore factory defaults

    transferMotor.setInverted(true);

    //transferLimitCurrentConfigs.withStatorCurrentLimit(TransferConstants.transferCurrentLimit);
    //transferLimitCurrentConfigs.withStatorCurrentLimitEnable(true);
    //transferConfigurator.apply(transferLimitCurrentConfigs);

    
  }
}

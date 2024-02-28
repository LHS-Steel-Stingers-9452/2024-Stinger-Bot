// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.transfer;


//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.TransferConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.NeutralOut;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Transfer extends SubsystemBase {
  /** Creates a new Launcher. */

  private final TalonFX transferMotor;

  private DigitalInput photoSensor;

  private TalonFXConfigurator transferConfigurator;
  private TalonFXConfiguration transferConfig = new TalonFXConfiguration();

  public Transfer() {
    transferMotor = new TalonFX(TransferConstants.transferID);

    photoSensor = new DigitalInput(DIOConstants.photoSensDioPort);

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

  //Use photo sensor
  public boolean isNoteInTransfer(){
    if (photoSensor.get()) {
      return false;
    } else{
      return true;
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("transfer Speed(RPS)", getTransferSpeed());
    SmartDashboard.putBoolean("Is note?", isNoteInTransfer());
  }

  public void transferConfig(){
    //restore factory defaults
    transferConfigurator = transferMotor.getConfigurator();
    transferConfigurator.apply(transferConfig);

    transferMotor.setInverted(false);
    transferMotor.setNeutralMode(NeutralModeValue.Brake);

    //transferLimitCurrentConfigs.withStatorCurrentLimit(TransferConstants.transferCurrentLimit);
    //transferLimitCurrentConfigs.withStatorCurrentLimitEnable(true);
    //transferConfigurator.apply(transferLimitCurrentConfigs);

    
  }
}

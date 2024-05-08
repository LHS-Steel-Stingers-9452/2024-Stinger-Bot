// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
//import frc.robot.Constants.TransferConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;


public class Intake extends SubsystemBase {
  /** Creates a new intake. */
  private final CANSparkMax intakeMotor;
  private final RelativeEncoder encoder;

  public Intake() {
    intakeMotor = new CANSparkMax(IntakeConstants.intakeID, MotorType.kBrushless);
    encoder = intakeMotor.getEncoder();

    configIntakeMotor();
  }

  //values range from -1 to 1
  public void setIntakeMotorSpeed(double intakeSpeed){
    intakeMotor.set(intakeSpeed);
  }

  public void stopIntake(){
     intakeMotor.set(0);
  }

  public double getIntakeSpeed(){
    return encoder.getVelocity();
  }


  @Override
  public void periodic() {
    //This method will be called once per scheduler
    SmartDashboard.putNumber("Intake Speed (RPM)", Math.abs(getIntakeSpeed()));
  }


  public void configIntakeMotor(){
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(true);
    intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    intakeMotor.setSmartCurrentLimit(30);//current limit
    intakeMotor.enableVoltageCompensation(IntakeConstants.voltageComp);
    encoder.setVelocityConversionFactor(0.2);
    intakeMotor.burnFlash();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkPIDController;
//import com.revrobotics.CANSparkBase.ControlType;


public class Intake extends SubsystemBase {
  /** Creates a new intake. */
  private final CANSparkMax intakeMotor;
  private final RelativeEncoder encoder;
  //private final SparkPIDController intakePIDController; 

  public Intake() {
    intakeMotor = new CANSparkMax(IntakeConstants.intakeID, MotorType.kBrushless);
    encoder = intakeMotor.getEncoder();
    //intakePIDController = intakeMotor.getPIDController();


    configIntakeMotor();
  }

  //values range fomr -1 to 1
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
    SmartDashboard.putNumber("Intake Speed (RPM)", getIntakeSpeed());
    SmartDashboard.putNumber("Intake Speed (RPS)", (getIntakeSpeed() * (1/60)));
  }


  public void configIntakeMotor(){
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(false);
    intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    intakeMotor.enableVoltageCompensation(IntakeConstants.voltageComp);
    intakeMotor.burnFlash();
  }
}

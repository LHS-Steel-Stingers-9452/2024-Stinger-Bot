// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climbers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climbers extends SubsystemBase {
  /** Creates a new Climbers. */
   private final CANSparkMax leftClimber;
    private final CANSparkMax rightCLimber;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

  public Climbers() {
    leftClimber = new CANSparkMax(24, MotorType.kBrushless);
    rightCLimber = new CANSparkMax(25, MotorType.kBrushless);

    leftClimber.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightCLimber.setIdleMode(CANSparkMax.IdleMode.kBrake);

    leftEncoder = leftClimber.getEncoder();
    rightEncoder = rightCLimber.getEncoder();   
    
    leftClimber.setInverted(true);
    rightCLimber.setInverted(false);

    leftEncoder.setPositionConversionFactor((1/60) * 360);
    rightEncoder.setPositionConversionFactor((1/60) * 360);

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    //leftClimber.setSmartCurrentLimit(15);
    //rightCLimber.setSmartCurrentLimit(15);
    leftClimber.setSmartCurrentLimit(60, 15);
    rightCLimber.setSmartCurrentLimit(60, 15);  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("leftClimber output current[amps]", leftClimber.getOutputCurrent());
    SmartDashboard.putNumber("rightClimber output current[amps]", rightCLimber.getOutputCurrent());
  }

  public void moveup(){
    leftClimber.set(1);
    rightCLimber.set(1);
  }

  public void moveDown(){
    leftClimber.set(-1);
    rightCLimber.set(-1);
  }

  public void stopClimber(){
    leftClimber.set(0);
    rightCLimber.set(0);
  }
}

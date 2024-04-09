// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climbers;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.Setpoints;

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
    rightCLimber.setInverted(true);

    leftEncoder.setPositionConversionFactor(1/60);
    rightEncoder.setPositionConversionFactor(1/60);

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    leftClimber.setSoftLimit(SoftLimitDirection.kReverse, 0);
    rightCLimber.setSoftLimit(SoftLimitDirection.kReverse, 0);

    //leftClimber.setSoftLimit(SoftLimitDirection.kForward, 0);
    //rightCLimber.setSoftLimit(SoftLimitDirection.kForward, 0);

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("leftClimber Pos[Rot]", leftEncoder.getPosition());
    SmartDashboard.putNumber("rightClimber Pos[Rot]", rightEncoder.getPosition());
  }

  public void moveup(){
    leftClimber.set(1);
    rightCLimber.set(1);
  }

  public void moveDown(){
    leftClimber.set(-0.85);
    rightCLimber.set(-0.85);
  }

  public void stopClimber(){
    leftClimber.set(0);
    rightCLimber.set(0);
  }
}

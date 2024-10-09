// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climbers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClimberConstants.*;

public class Climbers extends SubsystemBase {
  /** Creates a new Climbers. */
    private final CANSparkMax rightCLimber;


    //private final SparkPIDController leftPIDController;
    private final SparkPIDController rightPIDController;

    //private final  RelativeEncoder leftRelative;
    private final RelativeEncoder rightRelative;
    
  public Climbers() {
    //Climb Motors

    //leftClimber = new CANSparkMax(24, MotorType.kBrushless);
    rightCLimber = new CANSparkMax(25, MotorType.kBrushless);

    /*Climb Feedback */
    //leftPIDController = leftClimber.getPIDController();
    rightPIDController = rightCLimber.getPIDController();

    //leftRelative = leftClimber.getEncoder();
    rightRelative = rightCLimber.getEncoder();

    //leftClimberConfig();
    rightClimberConfig();
  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Left Climber pos[Rot]", getLeftClimberPos());
    SmartDashboard.putNumber("Right Climber pos[Rot]", getRightClimberPos());
  }

  public void climbUpTest(){
    //leftClimber.set(1);
    rightCLimber.set(1);
  }

   public void climbDownTest(){
    //leftClimber.set(-1);
    rightCLimber.set(-1);
  }

  public void stopClimber(){
    //leftClimber.set(0);
    rightCLimber.set(0);
  }


  /* 
  private void leftClimberConfig(){
    leftClimber.restoreFactoryDefaults();

    leftClimber.setInverted(true);
    leftClimber.setIdleMode(CANSparkMax.IdleMode.kBrake);
    leftClimber.setSmartCurrentLimit(40);

    leftClimber.enableVoltageCompensation(12);

    //account for gear ratio
    leftRelative.setPositionConversionFactor((1/60));
    
    leftPIDController.setP(kP);
    leftPIDController.setI(kI);
    leftPIDController.setD(kD);
    leftClimber.burnFlash();

    leftRelative.setPosition(0);

  }
  */

  private void rightClimberConfig(){
     rightCLimber.restoreFactoryDefaults();

    rightCLimber.setInverted(false);
    rightCLimber.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightCLimber.setSmartCurrentLimit(40);

    rightCLimber.enableVoltageCompensation(12);

    //account for gear ratio
    rightRelative.setPositionConversionFactor((1/60));
    
    rightPIDController.setP(kP);
    rightPIDController.setI(kI);
    rightPIDController.setD(kD);

    rightCLimber.burnFlash();

    rightRelative.setPosition(0);

  }

  /**
   * set individual climer positions[Motor Rotations]
   * 
   * @param rightClimb right climber setpoint
   */
  public void setClimberSetpoint(double rightClimb){
    rightPIDController.setReference(rightClimb, ControlType.kPosition);
  }



  public double getLeftClimberPos(){
    return rightRelative.getPosition();
  }

  public double getRightClimberPos(){
    return rightRelative.getPosition();
  }
}
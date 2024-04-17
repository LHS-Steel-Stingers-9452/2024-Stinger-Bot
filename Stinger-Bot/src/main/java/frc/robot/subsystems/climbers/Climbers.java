// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climbers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClimberConstants.*;

public class Climbers extends SubsystemBase {
  /** Creates a new Climbers. */
    private final CANSparkMax leftClimber;
    private final CANSparkMax rightCLimber;


    private final SparkPIDController leftPIDController;
    private final SparkPIDController rightPIDController;
    private final SparkAbsoluteEncoder leftAbsolute;
    private final SparkAbsoluteEncoder rightAbsolute;
    


  public Climbers() {
    //Climb Motors
    leftClimber = new CANSparkMax(24, MotorType.kBrushless);
    rightCLimber = new CANSparkMax(25, MotorType.kBrushless);


    /*Climb Feedback */
    leftPIDController = leftClimber.getPIDController();
    rightPIDController = rightCLimber.getPIDController();


    leftAbsolute = leftClimber.getAbsoluteEncoder(Type.kDutyCycle);
    rightAbsolute = rightCLimber.getAbsoluteEncoder(Type.kDutyCycle);

    leftClimberConfig();
    rightClimberConfig();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Climber pos[deg]", getLeftClimberPos());
    SmartDashboard.putNumber("Right Climber pos[deg]", getRightClimberPos());
  }

  public void climbUpTest(){
    leftClimber.set(.6);
    rightCLimber.set(.6);
  }

   public void climbDownTest(){
    leftClimber.set(-1);
    rightCLimber.set(-1);
  }

  public void stopClimber(){
    leftClimber.set(0);
    rightCLimber.set(0);
  }



  private void leftClimberConfig(){
    leftClimber.restoreFactoryDefaults();

    leftClimber.setInverted(false);
    leftClimber.setIdleMode(CANSparkMax.IdleMode.kBrake);

    //make sure this is positive when turning up
    leftAbsolute.setInverted(false);

    leftClimber.enableVoltageCompensation(12);

    leftPIDController.setFeedbackDevice(leftAbsolute);
    
    leftPIDController.setP(0);
    leftPIDController.setI(0);
    leftPIDController.setD(0);
    leftClimber.burnFlash();


  }


  private void rightClimberConfig(){
     rightCLimber.restoreFactoryDefaults();

    rightCLimber.setInverted(true);
    rightCLimber.setIdleMode(CANSparkMax.IdleMode.kBrake);

    //make sure this is positive when turning up
    rightCLimber.setInverted(true);

    rightCLimber.enableVoltageCompensation(12);

    rightPIDController.setFeedbackDevice(rightAbsolute);
    
    rightPIDController.setP(kP);
    rightPIDController.setI(kI);
    rightPIDController.setD(kD);

    rightCLimber.burnFlash();


  }


  public void setClimberSetpoint(double leftClimb, double rightClimb){
    leftPIDController.setReference(leftClimb, ControlType.kPosition);
    rightPIDController.setReference(rightClimb, ControlType.kPosition);
  }


  public double getLeftClimberPos(){
    return leftAbsolute.getPosition();
  }

  public double getRightClimberPos(){
    return rightAbsolute.getPosition();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climbers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climbers extends SubsystemBase {
  /** Creates a new Climbers. */
    private final CANSparkMax leftClimber;
    private final SparkAbsoluteEncoder leftAbsolute;

    /*Right climber*/
    private final CANSparkMax rightCLimber;
    private final SparkAbsoluteEncoder rightAbsolute;


  public Climbers() {
    /*left climber */
    leftClimber = new CANSparkMax(24, MotorType.kBrushless);
    leftAbsolute = leftClimber.getAbsoluteEncoder(Type.kDutyCycle);


    /*right climber */
    rightCLimber = new CANSparkMax(25, MotorType.kBrushless);
    rightAbsolute = rightCLimber.getAbsoluteEncoder(Type.kDutyCycle);

    leftClimberConfig();
    rightClimberConfig();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Climber pos[deg]", leftClimberPos());
    SmartDashboard.putNumber("Right Climber pos[deg]", rightAbsolute.getPosition());

  }

  public void moveup(){
    //left climber controls
    //if approaching max setpoint slow down and stop once withn 20 degrees 
    /* 
    if (Math.abs(ClimberConstants.maxClimberHeight - leftClimberPos()) < 90){
      leftClimber.set(.5);
    } else if(Math.abs(ClimberConstants.maxClimberHeight - leftClimberPos()) < 20){
      leftClimber.set(0);
    } else{
      leftClimber.set(1);
    }

    //right climber controls
    //if approaching max setpoint slow down and once withn 20 degrees 
    if (Math.abs(ClimberConstants.maxClimberHeight - rightClimberPos()) <= 90){
      rightClimber.set(.5);
    } else if(Math.abs(ClimberConstants.maxClimberHeight - rightClimberPos()) <= 20){
      leftClimber.set(0);
    } else{
      rightCLimber.set(1);
    */
    leftClimber.set(1);
    rightCLimber.set(1);
  }

  public void moveDown(){
    //left climber controls
    /*
    //if approaching min setpoint slow down and stop once withn 20 degrees 
    
    if (leftClimberPos() <= 90){
      leftClimber.set(-.5);
    } else if(leftClimberPos() <= 20){
      leftClimber.set(0);
    } else{
      leftClimber.set(-1);
    }
    
    //right climber controls
    //if approaching max setpoint slow down and once withn 20 degrees 
    if (rightClimberPos() < 90){
      rightCLimber.set(-.5);
    } else if(rightClimberPos() < 20){
      leftClimber.set(0);
    } else{
      rightCLimber.set(-1);
    }
    */

    
    leftClimber.set(-1);
    rightCLimber.set(-1);
  }

  public void stopClimber(){
    leftClimber.set(0);
    rightCLimber.set(0);
  }

  private void leftClimberConfig(){
    leftClimber.setIdleMode(CANSparkMax.IdleMode.kBrake);
    leftClimber.setInverted(true);

    //Don't event include gear ration since encoder is on the output shaft
    leftAbsolute.setPositionConversionFactor(360);
    //make sure this is positive when turning up
    leftAbsolute.setInverted(true);


    leftAbsolute.setZeroOffset(0);
  }

  private void rightClimberConfig(){
    rightCLimber.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightCLimber.setInverted(false);

    //Don't event include gear ration since encoder is on the output shaft
    rightAbsolute.setPositionConversionFactor(360);
    //make sure this is positive when turning up
    rightAbsolute.setInverted(false);
    rightAbsolute.setZeroOffset(0);
  }

  public double leftClimberPos(){
    return leftAbsolute.getPosition();
  }

  public double rightClimberPos(){
    return rightAbsolute.getPosition();
  }
}

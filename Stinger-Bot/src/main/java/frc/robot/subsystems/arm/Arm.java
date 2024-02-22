// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Util.Setpoints;
import frc.robot.Util.TunableNumber;



public class Arm extends ProfiledPIDSubsystem {
  /** Creates a new Arm. */

  private static TunableNumber tuneArmSetPoint = new TunableNumber("Tunable Arm SetPoint", 0.0);

  private final TalonFX leadMotor = new TalonFX(ArmConstants.leadID);
  private final TalonFX followMotor = new TalonFX(ArmConstants.followID);

  private VoltageOut voltageOutput = new VoltageOut(0.0);

  private DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(DIOConstants.encoderDioPort);

  private ArmFeedforward armFeedforward = new ArmFeedforward(
    ArmConstants.armKS, 
    ArmConstants.armKG, 
    ArmConstants.armKV,
    ArmConstants.armKA);

  /*create our own TP State object so a new one is not created on every setGoal() call*/
  private TrapezoidProfile.State tpState = new TrapezoidProfile.State(0.0, 0.0);

  /*working (current setpoint)*/
  private double armSetPoint;

  /*working current tolerance*/
  private double tolerance;

  TunableNumber tempDegree = new TunableNumber("Arm go to Degrees", 0.0);

  /*
   * Constructor
   */
  public Arm() {
    super(
        /*Creates the Trapezoidal motion profile controller */
        new ProfiledPIDController(
            ArmConstants.armP,
            ArmConstants.armI,
            ArmConstants.armD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(ArmConstants.armCruise, armAcceleration)));

    //Start arm at rest in stowed/intakepositon position
    updateArmSetPoint(RobotConstants.STOWED);

    //Config Duty Cycle Range for the encoders
    absoluteEncoder.setDutyCycleRange(ArmConstants.encoderMin, ArmConstants.encoderMax);

    //config for motors
    var leadMotorConfig = new TalonFXConfiguration();
    var followMotorMotorConfig = new TalonFXConfiguration();

    //set the output mode to brake

    //set th motors Neutral Deadband

    //set the turning direction

    /*
     * Appply the configurations to the motors
     * and set one to follow the other in the same direction
     */


    //Put controls for the PID controller on the dashboard
    if (RobotConstants.isArmTurningMode) SmartDashboard.putData(this.m_controller);
  }

  @Override
  public void periodic(){
    //Make sure the parent controller gets to do its own updates
    super.periodic();

    //Validate current encoder reading; stop motors if out of range
    //code here

    SmartDashboard.putBoolean("Is Arm at Setpoint?", isArmAtSetPoint());

    if (RobotConstants.isTuningMode){
      SmartDashboard.putNumber("Arm Setpoint", armSetPoint);
      SmartDashboard.putNumber("Raw Arm Encoder", getAbsPos());
      SmartDashboard.putNumber("Arm Angle Uncorrected", dutyCycleToDegrees(getAbsPos()));
      SmartDashboard.putNumber("Current Arm Angle", armSetPointDegrees());
      SmartDashboard.putNumber("Arm Error", getArmError());

    }
  }


  /**
   * Consumes the output from the ProfiledPIDController.
   * 
   * The PID will automatically call this method from its periodic()
   * block, and pass it the computed output of the control loop.
   * 
   * @param output the output of the ProfiledPIDController
   * @param setpoint the setpoint state of hte ProfiledPIDController
   */


  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here

    //correct passed in current setpoint before calculating feedforward
    double correctedPosition = correctedArmRadiansForFeedFWD(setpoint.position);

    //cakcuulkate hte feedforward using the corrected setpoint
    double feedforward = armFeedforward.calculate(correctedPosition, setpoint.velocity);

    if (RobotConstants.isArmTurningMode){
      

    }
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }


  
}

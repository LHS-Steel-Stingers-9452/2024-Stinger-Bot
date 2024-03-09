// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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
            new TrapezoidProfile.Constraints(ArmConstants.armCruise, ArmConstants.armAcceleration)));

    //Start arm at rest in STOWED position
    updateArmSetPoint(RobotConstants.STOWED);

    //Config Duty Cycle Range for the encoders
    absoluteEncoder.setDutyCycleRange(ArmConstants.encoderDutyCycleMin, ArmConstants.encoderDutyCycleMax);

    //absoluteEncoder.reset();
    

    //config for motors
    var leadMotorConfig = new TalonFXConfiguration();
    var followMotorMotorConfig = new TalonFXConfiguration();

    //Set the output mode to brake
    leadMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    followMotorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    //Set the motor's Neutral Deadband
    leadMotorConfig.MotorOutput.DutyCycleNeutralDeadband = ArmConstants.neutralDeadBand;
    leadMotorConfig.MotorOutput.DutyCycleNeutralDeadband = ArmConstants.neutralDeadBand;

    /* Set the turning direction */
    leadMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    /*
     * Apply the configurations to the motors, and set one to follow the other in
     * the same direction
     */
    leadMotor.getConfigurator().apply(leadMotorConfig);
    followMotor.getConfigurator().apply(followMotorMotorConfig);
    followMotor.setControl(new Follower(leadMotor.getDeviceID(), true));

    // optimize StatusSignal rates for the Talons
    //leadMotor.getSupplyVoltage().setUpdateFrequency(4);
    //leadMotor.optimizeBusUtilization();
    //followMotor.getSupplyVoltage().setUpdateFrequency(4);
    //followMotor.optimizeBusUtilization();


    //Put controls for the PID controller on the dashboard
    if (RobotConstants.isArmTurningMode) SmartDashboard.putData(this.m_controller);
  }

  @Override
  public void periodic(){
    //Make sure the parent controller gets to do its own updates
    super.periodic();

    SmartDashboard.putBoolean("Is Arm at Setpoint?", isArmAtSetPoint());

    if (RobotConstants.isTuningMode){
      SmartDashboard.putNumber("Arm Setpoint", armSetPoint);
      SmartDashboard.putNumber("Raw Arm Encoder", getAbsPos());
      SmartDashboard.putNumber("Arm Angle Uncorrected", dutyCycleToDegrees(getAbsPos()));
      SmartDashboard.putNumber("Current Arm Angle (Degrees)", getArmDegrees());
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
      SmartDashboard.putNumber("Arm corrected FF position", correctedPosition);
      SmartDashboard.putNumber("Arm PID output", output);
      SmartDashboard.putNumber("Arm Feed Forward Output", feedforward);
    }

    leadMotor.setControl(voltageOutput.withOutput(output + feedforward));
  }

  /**
     * Returns the measurement of the process variable used by the
     * ProfiledPIDController.
     * 
     * The PIDSubsystem will automatically call this method from its periodic()
     * block, and pass the returned value to the control loop.
     * 
     * @return the measurement of the process variable, in this case, the Arm angle,
     * in radians corrected to 0.0 at the STOWED position
  */
  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getArmRadians();
  }

  /**
   * Update the PID controller's current Arm setpoint and tolerance
   * 
   * @param setpoints - the desired position as a Setpoints object
   */

   public void updateArmSetPoint(Setpoints setpoints){
    //convert degrees to radians and set the profile goal
    armSetPoint = setpoints.arm;
    tolerance = setpoints.tolerance;
    //Arm setpoint must be passed in radians
    tpState.position = degreesToRadians(setpoints.arm);
    setGoal(tpState);

    //Display requested Arm State to dashboard
    Setpoints.displayArmState(setpoints.state);
   }

   /**
    * Update the PID controller's current Arm setpoint in degrees
    * @param degrees - the desired Arm position in degrees
    */
   public void updateArmInDegrees(double degrees){
    //Convert degrees to radians and set the profile goals
    armSetPoint = degrees;
    tpState.position = degreesToRadians(degrees);
    setGoal(tpState);
  }

  public double getDegrees(){
    //Get the SmartDashboard
    return tempDegree.get();
  }

  /**
   * Override the enalbe() method so we can set the goal to the current position
   * 
   * The super method resets the controller and sets its current setpoint to the 
   * current position, but does not reset the goal, which will cause the Arm
   * to jump from the current position to the old goal.
   */
  @Override
  public void enable(){
    super.enable();
    armSetPoint = getArmDegrees();
    setGoal(armSetPoint);
  }

  /**
   * Get the current Arm position error (in degrees)
   */
  public double getArmError(){
    return Math.abs(armSetPoint - getArmDegrees());
  }

  /**Check if Arm is at the setpoint or within tolerance*/
  public boolean isArmAtSetPoint(){
    return getArmError() < tolerance;
  }

  /**Drive the Arm directly by providing a supply voltage value*/
  public void setArmVoltage(double voltage){
    leadMotor.setControl(voltageOutput.withOutput(voltage));
  }

  /**Set the lead motor to the Neutral state (no output) */
  public void stopMotors() {
    leadMotor.setControl(new NeutralOut());
  }

  /**Returns the current enocder absolute value in DutyCycle units (~0 -> ~1) */
  public double getAbsPos(){
    return absoluteEncoder.getAbsolutePosition();
  }

  /**Converts DutyCycle units to Degrees */
  public double dutyCycleToDegrees(double dutyCyclePos){
    return dutyCyclePos * 360;
  }

  /**Converts the current encoder reading to Degrees, and corrects relative to a
   * STOWED position of zero
   */
  public double getArmDegrees(){
    return dutyCycleToDegrees(getAbsPos()) - ArmConstants.armStartingOffset;
  }

  /**Converts DutyCycle units to Radians */
  public double dutyCycleToRadians(double dutyCyclePos){
    return dutyCyclePos * 2.0 * Math.PI;
  }

  /**Converts Degrees to Radians */
  public double degreesToRadians(double degrees){
    return (degrees * Math.PI) / 180.0;
  }

  /**Convert the current encoder reading to Degrees, and corrects relative
   * to a STOWED position of zero
   */
  public double getArmRadians(){
    return dutyCycleToRadians(getAbsPos()) - degreesToRadians(ArmConstants.armStartingOffset);
  }

  /**
   * Takes a position in radians relative to STOWED, and corrects it
   * to be relative to a HORIZONTAL position of zero.
   * This is used for Feedforward only, where we accoint for gravity using a cosine function
   */
  public double correctedArmRadiansForFeedFWD(double position){
    return position - degreesToRadians(ArmConstants.armHorizontalOffset - ArmConstants.armStartingOffset);
  }

  /*
   * Command Factories
   */

   /**To position for Intake, move Arm to INTAKE position */
  public Command prepareForIntakeCommand(){
    return new RunCommand(() -> this.updateArmSetPoint(RobotConstants.INTAKE), this)
    .until(() -> this.isArmAtSetPoint());
  }

  /**To tune the lookup table using SmartDashboard */
  public Command tuneArmSetPointCommand() {
    return new RunCommand(()-> this.updateArmInDegrees(tuneArmSetPoint.get()), this)
        .until(()->this.isArmAtSetPoint());
}

public Command moveToDegreeCommand(){
  return new RunCommand(() -> this.updateArmInDegrees(this.getDegrees()));
}
}

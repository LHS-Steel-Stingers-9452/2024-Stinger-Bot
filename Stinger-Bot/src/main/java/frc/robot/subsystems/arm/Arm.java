package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// Imports go here
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ArmConstants.kCurrentLimit;
import static frc.robot.Constants.ArmConstants.kErrorTolerance;
import static frc.robot.Constants.DIOConstants.encoderDioPort;
import static frc.robot.Constants.ArmConstants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Arm extends SubsystemBase {
  // Misc variables for specific subsystem go here

  // Enum representing all of the states the subsystem can be in
  public enum PivotStates {
    DefaultState,
    AmpState,
    ShooterState,
    MidState,
    CustomState
    
  }

  public static PivotStates ArmCurrentState;
  public static PivotStates ArmRequestedState;

  GenericEntry armPosition;

  // You may need more than one motor
  private final TalonFX leadKraken = new TalonFX(leadID);
  private final TalonFX followKraken = new TalonFX(followID);
  //private final DutyCycleEncoder absEncoder = new DutyCycleEncoder(encoderDioPort);

  DoubleSupplier angle; 
  private final MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(0);
  // Unit default for TalonFX libraries is rotations
  private double desiredPosition = 0;

  //Constructor
  public Arm(DoubleSupplier angle) {

    this.angle = angle;
    // Misc setup goes here

    armPosition = Shuffleboard.getTab("Arm").add("ArmPosition[Rotations]", 0).getEntry();

    //based on phoenix example
    var talonFXConfigs = new TalonFXConfiguration();
    // These will be derived experimentally but in case you are wondering
    // How these terms are defined from the TalonFX docs
    // kS adds n volts to overcome static friction
    // kG
    // kA
    // kP outputs 12 volts when the positional error is 12/n rotations
    // kI adds n volts per second when the positional error is 1 rotation
    // kD outputs n volts when the velocity error is 1 rotation per second
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = kS;
    //slot0Configs.kG = kG;
    //slot0Configs.kA = kA;
    slot0Configs.kV = kV;
    slot0Configs.kP = kP;
    slot0Configs.kI = kI;
    slot0Configs.kD = kP;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = kCruiseVelocity;
    // vel/acc = time to reach constant velocity
    motionMagicConfigs.MotionMagicAcceleration = kAcceleration;
    // acc/jerk = time to reach constant acceleration
    motionMagicConfigs.MotionMagicJerk = kJerk;



    var motorOutputConfigs = talonFXConfigs.MotorOutput;
    if (kClockwisePositive)
      motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    else motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    var feedbackConfigs = talonFXConfigs.Feedback;
    feedbackConfigs.SensorToMechanismRatio = kSensorToMechanismGearRatio;//accounting for output with gear ratios

    var currentConfigs = talonFXConfigs.CurrentLimits;
    currentConfigs.StatorCurrentLimitEnable = true;
    currentConfigs .StatorCurrentLimit = kCurrentLimit;

    leadKraken.getConfigurator().apply(talonFXConfigs);
    followKraken.getConfigurator().apply(talonFXConfigs);

    ArmCurrentState = PivotStates.DefaultState;
    ArmRequestedState = PivotStates.DefaultState;

    // if we design the robot with a proper resting position in mind
    // this should be the only initilization necessary
    // no firstTime2 :)
    leadKraken.setPosition(0);
    //followKraken.setPosition(0);
  }

  @Override
  public void periodic() {

    armPosition.setDouble(getError());

    switch (ArmRequestedState) {
      case DefaultState:
        desiredPosition = 0;
        break;
      case AmpState:
        desiredPosition = 0.22;//tune
        break;
      case ShooterState:
        desiredPosition = MathUtil.clamp(angle.getAsDouble(), 0, .25);//tune
        break;
      case MidState:
        desiredPosition = 0.044;
        break;
    }
 
    runControlLoop();

    if (getError() < kErrorTolerance)
      ArmCurrentState = ArmRequestedState;
    else
      ArmCurrentState = PivotStates.CustomState;  
  }

  public void runControlLoop() {
    leadKraken.setControl(request.withPosition(desiredPosition));
    followKraken.setControl(new Follower(leadID , true));
  }

  private double getPosition() {
    return leadKraken.getPosition().getValue();
  }

  public double getError() {
    return Math.abs(getPosition() - desiredPosition);
  }
 
  // example of a "setter" method
  public void requestState(PivotStates requestedState) {
    ArmRequestedState = requestedState;
  }
 
  // example of a "getter" method
  public PivotStates getCurrentState() {
    return ArmCurrentState;
  }
}
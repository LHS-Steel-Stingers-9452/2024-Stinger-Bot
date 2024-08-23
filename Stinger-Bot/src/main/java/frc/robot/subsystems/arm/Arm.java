package frc.robot.subsystems.arm;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

import static frc.robot.Constants.ArmConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Arm extends SubsystemBase {

  GenericEntry position;
  GenericEntry posError;
  GenericEntry atPosition;

  private final TalonFX leadKraken;
  private final TalonFX followKraken;

  private final MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(0);

  private TalonFXConfiguration motorConfig = new TalonFXConfiguration();

  // Unit default for TalonFX libraries is rotations
  private double desiredPosition;

  //Constructor
  public Arm() {

    leadKraken = new TalonFX(leadID);
    followKraken = new TalonFX(followID);

    desiredPosition = 0;

    position = Shuffleboard.getTab("Arm").add("Arm Pos[Rotations]", 0).getEntry();
    posError = Shuffleboard.getTab("Arm").add("Arm Error[Rotations]", 0).getEntry();
    atPosition = Shuffleboard.getTab("Arm").add("Arm at requested Pos?", 0).getEntry();

    // These will be derived experimentally but in case you are wondering
    // How these terms are defined from the TalonFX docs
    // kS adds n volts to overcome static friction
    // kP outputs 12 volts when the positional error is 12/n rotations
    // kI adds n volts per second when the positional error is 1 rotation
    // kD outputs n volts when the velocity error is 1 rotation per second
    var slot0Configs = motorConfig.Slot0;
    slot0Configs.kS = kS;
    slot0Configs.kV = kV;
    slot0Configs.kP = kP;
    slot0Configs.kI = kI;
    slot0Configs.kD = kD;
    //TODO - add kG constant
    //slot0Configs.kG = kG;

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = kCruiseVelocity;
    // vel/acc = time to reach constant velocity
    motorConfig.MotionMagic.MotionMagicAcceleration = kAcceleration;
    // acc/jerk = time to reach constant acceleration
    motorConfig.MotionMagic.MotionMagicJerk = kJerk;



    if (kClockwisePositive) {
      motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    } else {
      motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfig.Feedback.SensorToMechanismRatio = kSensorToMechanismGearRatio;//accounting for gear ratios

    motorConfig.CurrentLimits.StatorCurrentLimitEnable = kEnableCurrentLimit;
    motorConfig.CurrentLimits .StatorCurrentLimit = kCurrentLimit;

    leadKraken.getConfigurator().apply(motorConfig);
    followKraken.getConfigurator().apply(motorConfig);

    //TODO - Test if setting follower in Contructor is effective 
    followKraken.setControl(new Follower(leadID , true));

    //NOTE - Set's Zero
    leadKraken.setPosition(0);
  }

  //SECTION - setters
  public void setPosition(Double desiredPosition) {
    //leadKraken.setVoltage(.45);
    this.desiredPosition = desiredPosition;
    leadKraken.setControl(request.withPosition(desiredPosition));
  }

  public void setCoast(){
    leadKraken.setControl(new CoastOut());
  }

  public void setBreak(){
    leadKraken.setControl(new StaticBrake());
  }

  //SECTION - getters
  private double getPosition() {
    return leadKraken.getPosition().getValue();
  }

  public double getDesiredPosition() {
    return desiredPosition;
  }

  public double getError() {
    return Math.abs(getPosition() - desiredPosition);
  }

  @Override
  public void periodic() {
    position.setDouble(getPosition());
    posError.setDouble(getError());
    atPosition.setBoolean(atRequestedPos());
  }

  //NOTE - kErrorTolerance value may need update
  public boolean atRequestedPos(){
    if(getError() <= ArmConstants.kErrorTolerance){
      return true;
    } else{
      return false;
    }
  }
}
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class RobotArm extends SubsystemBase {
  /** Creates a new RobotArm. */

  //Creates Motor variables
  private final CANSparkMax ArmLeadMotor;
  private final CANSparkMax ArmFollowerMotor;

  //Creates Motor group variables
  private MotorControllerGroup ArmMotorGroup;

  //Creates Motor Encoders
  private AbsoluteEncoder AbsoluteEncoder;

  //Creates Forward and Turn Vars For Speed Input
  private double forward;
  private double turn; 

  // Create Filters For Slew Rate Limiting
  private SlewRateLimiter pivotFilter;

  public RobotArm(Object CANSparkMaxLowLevel) {
      //Initialize Motors
      ArmLeadMotor = new CANSparkMax(ArmConstants.ARM_MOTOR_CAN_ID_LEFT,CANSparkMaxLowLevel.kBrushless);
      ArmFollowerMotor = new CANSparkMax(ArmConstants.ARM_MOTOR_CAN_ID_LEFT,CANSparkMaxLowLevel.MotorType.kBrushless);

      //Slew Rate Limiter
      pivotFilter = new SlewRateLimiter(ArmConstants.SLEW_RATE_DRIVE_POSITIVE,ArmConstants.SLEW_RATE_DRIVE_NEGATIVE, 0);

      //Restore As A Saftey Measure
      ArmLeadMotor.restoreFactoryDefaults();
      ArmFollowerMotor.restoreFactoryDefaults();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

//Incomplete Button Angle Presets
public final void ArmReset(Command ArmReset) {

}

public final void ButtonY() {

}
public final void ButtonB() {

}
public final void ButtonA() {

}
}
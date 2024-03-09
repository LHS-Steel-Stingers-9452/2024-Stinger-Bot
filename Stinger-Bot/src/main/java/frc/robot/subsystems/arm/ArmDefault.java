// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmDefault extends Command {
  /** Creates a new ArmDefault. */
  Arm armSub;
  BooleanSupplier m_joyMode;
  DoubleSupplier m_valueSrc;
  public ArmDefault(Arm arm, BooleanSupplier joyMode, DoubleSupplier valueSrc) {
    // Use addRequirements() here to declare subsystem dependencies.
    armSub = arm;
    m_joyMode = joyMode;
    m_valueSrc = valueSrc;
    addRequirements(armSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //If Operator is holding a particular button, then disable PID and go to Manual control mode
    if(m_joyMode.getAsBoolean()) {
      if(armSub.isEnabled()) armSub.disable();
      armSub.setArmVoltage(3.6 * m_valueSrc.getAsDouble());
    } else{
      if (!armSub.isEnabled()) armSub.enable();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.transfer.Transfer;

public class manualIntakeControl extends Command {
  /** Creates a new manualIntakeControl. */
  private Intake intakeSub;
  private Transfer transferSub;

  private double intakeSpeed;
  private double transferSpeed;

  public manualIntakeControl(Intake intakeSub, 
    double intakeSpeed, 
    Transfer transferSub, 
    double transferSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSub = intakeSub;
    this.transferSub = transferSub;
    this.intakeSpeed = intakeSpeed;
    this.transferSpeed = transferSpeed;

    addRequirements(intakeSub, transferSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double beSafeIntakeVal = intakeSpeed;
    double beSafeTransferVal = transferSpeed;
    intakeSub.setIntakeMotorSpeed(beSafeIntakeVal);
    transferSub.setTransferSpeed(beSafeTransferVal);
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

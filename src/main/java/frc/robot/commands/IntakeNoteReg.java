// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.transfer.Transfer;

public class IntakeNoteReg extends Command {
  /** Creates a new intakeNotereg. */
  private Intake intakeSub;
  private Transfer transferSub;

  private Double intakeSpeedSup;
  private Double transferSpeedSup;


  public IntakeNoteReg(
    Intake intakeSub, 
    Double intakeSpeedSup, 
    Transfer transferSub, 
    Double transferSpeedSup) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSub = intakeSub;
    this.intakeSpeedSup = intakeSpeedSup;
    this.transferSub = transferSub;
    this.transferSpeedSup = transferSpeedSup;

    addRequirements(intakeSub, transferSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double intakeSpeed = intakeSpeedSup;
    double transferSpeed = transferSpeedSup;

    intakeSub.setIntakeMotorSpeed(intakeSpeed);
    transferSub.setTransferSpeed(transferSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSub.stopIntake();
    transferSub.stopTransfer(); //command in command nono 
    
    //CommandManager.flashLeds();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return transferSub.isNoteInTransfer();
  }
}

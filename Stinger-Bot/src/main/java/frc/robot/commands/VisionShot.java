// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.launcher.Shooter;
import frc.robot.Util.LookUpTbl;

public class VisionShot extends Command {
      //shoot command
    /**
     * This Command should first make sure that the robot is in shooting mode meaning that the robot is aligned with speaker
     * we can utilize a simple public enum for this in  SwerveBase.java
     * option two is to just make a method where we see if the robot rotation difference with speaker is 0 and if it returns
     * true then proceed with setting arm 
     */
    private Arm armSub;
    private Shooter shooterSub;
    private LookUpTbl visionLookupTable;

  public VisionShot(Arm armSub, Shooter shooterSub){
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSub = armSub;
    this.shooterSub = shooterSub;

    addRequirements(armSub, shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  }
}

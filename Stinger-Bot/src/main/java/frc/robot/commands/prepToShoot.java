// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Util.Setpoints;
import frc.robot.Util.Setpoints.GameState;
import frc.robot.subsystems.arm.Arm;

public class prepToShoot extends Command {

    Setpoints m_setpoints;
    Arm m_armSubsystem;
    boolean m_isDone;

    /** Constructor - Creates a new prepareToShoot. */
    public prepToShoot(Setpoints setpoints, Arm armSub) {
    
        m_setpoints = setpoints;
        m_armSubsystem = armSub;

        addRequirements(armSub);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_isDone = false;
        if (!m_armSubsystem.isEnabled()) m_armSubsystem.enable();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

         m_armSubsystem.updateArmSetPoint(m_setpoints);

        // Exit once Arm is at setpoint and Shooter setpoint is != 0 and Shooter is up to speed 
        if (m_armSubsystem.isArmAtSetPoint()) {
            m_isDone = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Don't turn off anything unless we have been commanded to STOWED position
        if (m_setpoints.state == GameState.STOWED) {
            m_armSubsystem.disable();
        }

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_isDone;
    }
}
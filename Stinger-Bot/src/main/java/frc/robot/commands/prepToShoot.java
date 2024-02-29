// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Util.Setpoints;
import frc.robot.Util.Setpoints.GameState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.launcher.Shooter;

public class prepToShoot extends Command {

    Setpoints commandSetpoints;
    Arm armSub;
    Shooter shooterSub;
    //BooleanSupplier haveNote;
    boolean isDone;
    boolean runShooter;

    boolean haveNoteBeforeALways = true;

    /** Constructor - Creates a new prepareToShoot. */
    public prepToShoot(Setpoints setpoints, 
    //BooleanSupplier haveNote, 
    Arm armSub, Shooter shooterSub) {
    
       this.commandSetpoints = setpoints;
        this.armSub = armSub;
        this.shooterSub = shooterSub;
        //this.haveNote = haveNote;

        addRequirements(armSub, shooterSub);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        isDone = false;
        if (!armSub.isEnabled()) armSub.enable();

        //Is setpoint @zero?, if so don't check speed
        //[check that values are not zero]
        runShooter = (commandSetpoints.leftShooter != 0.0 || commandSetpoints.rightShooter != 0.0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        //send setpoints, using method overloading then run shooter
        //shooterSub.setShooterSetpoints(commandSetpoints);
        //shooterSub.runShooter();

        //bring arm to requested position
        //Don't require a Note if we are trying to stow arm
        if (haveNoteBeforeALways || commandSetpoints.state == GameState.STOWED) {
            armSub.updateArmSetPoint(commandSetpoints);
        }

        // Exit once Arm is at setpoint and Shooter setpoint is != 0 and Shooter is up to speed
        if (armSub.isArmAtSetPoint() 
        //&& (runShooter && shooterSub.areWheelsAtSpeed())
        ){//If not zero and wheels are at speed return true
            isDone = true;
            
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Don't turn off anything unless we have been commanded to STOWED position
        if (commandSetpoints.state == GameState.STOWED) {
            armSub.disable();
            shooterSub.stopShooter();
        }

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isDone;
    }
}
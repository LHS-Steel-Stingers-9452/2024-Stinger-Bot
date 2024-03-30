package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TransferConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Shooter;
import frc.robot.subsystems.transfer.Transfer;

public class autoCommands {
    public static Command intakeNote(Intake intake, Transfer transfer){
        //auto intake Note
        Command command = 
                new IntakeNoteReg(intake, IntakeConstants.intakeSpeed, transfer, TransferConstants.transferSpeed);
        return command;
    }

    public static Command shootNote(Shooter shooter, Transfer transfer){
        Command command = new SequentialCommandGroup(
            new InstantCommand(()-> shooter.setShooterSpeed(.50)).andThen(Commands.waitSeconds(.65)),
            new InstantCommand(()-> transfer.setTransferSpeed(TransferConstants.transferSpeed)).andThen(Commands.waitSeconds(.5)),
            new ParallelCommandGroup(new InstantCommand(()-> shooter.stopShooter())), new InstantCommand(() -> transfer.stopTransfer()));
        return command;

    }
            
    
}

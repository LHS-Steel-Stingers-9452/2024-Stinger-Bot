package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TransferConstants;
import frc.robot.commands.IntakeNoteReg;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.launcher.Shooter;
import frc.robot.subsystems.transfer.Transfer;

public class autoCommands {
    public static Command intakeNote(Intake intake, Transfer transfer){
        Command command = new ParallelDeadlineGroup(
                new IntakeNoteReg(intake, IntakeConstants.intakeSpeed, transfer, TransferConstants.transferSeed));
        return command;
    }

    public static Command shootNote(Shooter shooter, Transfer transfer){
        Command command = new ParallelDeadlineGroup(
            new InstantCommand(()-> transfer.isNoteInTransfer()),
            new InstantCommand(()-> shooter.setShooterSpeed(.50)).andThen(Commands.waitSeconds(2)),
            new InstantCommand(()-> transfer.setTransferSpeed(TransferConstants.transferSeed))
            );
        return command;


    }
            
    
}

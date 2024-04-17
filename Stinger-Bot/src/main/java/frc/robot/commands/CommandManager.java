package frc.robot.commands;

import javax.swing.DebugGraphics;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TransferConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.PivotStates;
import frc.robot.subsystems.climbers.Climbers;
import frc.robot.subsystems.drive.SwerveBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.launcher.Shooter;
import frc.robot.Constants.LauncherConstants;

public class CommandManager {

    public static Command intakeNote(Intake intake, Transfer transfer){
        //auto intake Note
        Command command = new ParallelCommandGroup(
            new InstantCommand(()-> intake.setIntakeMotorSpeed(IntakeConstants.intakeSpeed), intake),
            new InstantCommand(()-> transfer.setTransferSpeed(TransferConstants.transferSpeed), transfer));
        return command;
    }

    public static Command groundOuttake(Intake intake, Transfer transfer){
        //auto intake Note
        Command command = new ParallelCommandGroup(
            new InstantCommand(()-> intake.setIntakeMotorSpeed(IntakeConstants.intakeSpitSpeed), intake),
            new InstantCommand(()-> transfer.setTransferSpeed(TransferConstants.tranSpitSpeed), transfer));
        return command;
    }

    public static Command stopIntaking(Intake intake, Transfer transfer){
        Command command = new ParallelCommandGroup(
            new InstantCommand(() -> transfer.stopTransfer(), transfer),
            new InstantCommand(() -> intake.stopIntake(), intake));
        return command;
    }

    public static Command eStop(Intake intake, Transfer transfer, Arm arm, Shooter shooter){
        Command command = new ParallelCommandGroup(
            new InstantCommand(() -> transfer.stopTransfer(), transfer),
            new InstantCommand(() -> intake.stopIntake(), intake),
            new InstantCommand(() -> arm.requestState(PivotStates.DefaultState)),
            new InstantCommand(() -> shooter.stopShooter()));
        return command;
    }

    public static Command feedNote(Transfer transfer){
        Command command = 
            new InstantCommand(()-> transfer.setTransferSpeed(TransferConstants.transferSpeed), transfer);
        return command;
    }

    
    public static Command zeroGyro(SwerveBase swerveBase){
        Command command = 
            new InstantCommand(() -> swerveBase.zeroGyro());
        return command;
    }

    public static Command redReset(SwerveBase swerveBase, double degrees){
        Command command =
            new InstantCommand(
                ()-> swerveBase.setHeading(new Rotation2d(Units.degreesToRadians(degrees))));

        return command;
    }

    public static Command climberMove(Climbers climbers, double leftClimbSetpoint, double RightClimbSetPoint){
        Command command = 
            new ParallelCommandGroup(
                new InstantCommand(()-> climbers.setClimberSetpoint(leftClimbSetpoint, RightClimbSetPoint))
            );
        return command;
    }

    
}

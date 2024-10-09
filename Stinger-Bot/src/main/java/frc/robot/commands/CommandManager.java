package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TransferConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.SwerveBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.launcher.Shooter;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.climbers.Climbers;

public class CommandManager {

    public static Command intakeNote(Intake intake, Transfer transfer){
        //auto intake Note
        Command command = new ParallelCommandGroup(
           //new InstantCommand(()-> arm.requestState(PivotStates.CommunityShot)),
            new InstantCommand(()-> intake.setIntakeMotorSpeed(IntakeConstants.intakeSpeed), intake),
            new InstantCommand(()-> transfer.setTransferSpeed(TransferConstants.transferSpeed), transfer));
        return command;
    }

    public static Command spitNote(Intake intake, Transfer transfer){
        //reverse intake and transfer
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
            new InstantCommand(() -> arm.setPosition(0.0)),
            new InstantCommand(() -> shooter.instantStop()));
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

     public static Command climberMove(Climbers climbers,  double RightClimbSetPoint){
        Command command = 
            new ParallelCommandGroup(
                new InstantCommand(()-> climbers.setClimberSetpoint(RightClimbSetPoint))
            );
        return command;
    }

    //NOTE - Dead Code
    public static double limelightAim(){
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;

    @SuppressWarnings("resource")
    PIDController aimPidController = new PIDController(kP, 0, 0);

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = aimPidController.calculate(LimelightHelpers.getTX("Stinger_Cam"));

    // convert to radians per second for our drive method
    targetingAngularVelocity *= Swerve.maxAngleVelocity;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
    }

    //FIXME - Non responsive
    public static Command flashLeds(){
        Command command =
            new SequentialCommandGroup(
                new InstantCommand(()-> LimelightHelpers.setLEDMode_ForceBlink("Stinger_Cam")),
                Commands.waitSeconds(1),
                new InstantCommand(()-> LimelightHelpers.setLEDMode_ForceOff("Stinger_Cam")));
        return command;
    }

    
}

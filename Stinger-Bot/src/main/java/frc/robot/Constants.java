package frc.robot;

//import frc.robot.SwerveModuleConstants;

//import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
//import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
public class Constants {
    public final static class ControllerConstants{
        public static final int DRIVER_CONTROLLER_PORT = 0;
        //public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final double DEADBANDRANGE = 0.05;

        //public static final SlewRateLimiter translationFilter = new SlewRateLimiter(3);
        public static final double SLEW_RATE = 3;
    }

    public static final class Swerve{
        //Gyro ID
        public static final int PIGEON_ID = 13;

        //DriveTrain Constants
        public static final double TRACK_WIDTH = Units.inchesToMeters(24.75);
        public static final double WHEELBASE  = Units.inchesToMeters(24.75);
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.00);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        public static final double DRIVE_GEAR_RATIO = 6.75/1.0; //6.75:1
        public static final double ANGLE_GEAR_RATIO = (150.0/7.0)/1.0; //150/7:1

        //Encoder converstion factors
        public static final double DRIVE_ENCODER_POSITION_FACTOR = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;//Distance traveled
        public static final double DRIVE_ENCODER_VELOCITY_FACTOR = DRIVE_ENCODER_POSITION_FACTOR / 60.0;//M/S

        public static final double INTEGRATED_ANGLE_ENCODER_POSITION_FACTOR = WHEEL_CIRCUMFERENCE / ANGLE_GEAR_RATIO;//Distance traveled
        public static final double INTEGRATED_ANGLE_ENCODER_VELOCITY_FACTOR = INTEGRATED_ANGLE_ENCODER_POSITION_FACTOR / 60.0;//M/S

        public static final double CANCODER_VELOCITY_FACTOR = 360.0 / ANGLE_GEAR_RATIO;//Degrees per shaft rotation

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEELBASE/2, TRACK_WIDTH/2),
            new Translation2d(WHEELBASE/2, -TRACK_WIDTH/2),
            new Translation2d(-WHEELBASE/2, TRACK_WIDTH/2),
            new Translation2d(-WHEELBASE/2, -TRACK_WIDTH/2)

        );
        //meters/sec
        public static final double maxDriveSpeed = 4.0; //values are subject to change upon testing
        public static final double maxAngleVelocity = 4.0; 

        public static final int driveCurrentLimit = 70;
        public static final int angleCurrentLimit = 30;

        public static final boolean openLoop = true;
/* closed loop
        //feed forward values, need to be obtiaed though WPI charactarization tool
        public static final int driveKS;
        public static final int driveKV;
        public static final int driveKA;
*/
        //Drive Motor PID Values
        public static final double driveP = 0.10;
        public static final double driveI = 0.0;
        public static final double driveD = 0.0;
        public static final double driveFF = 0.0;

        //Angle Motor PID Values
        public static final double angleP = 0.01;
        public static final double angleI = 0.00;
        public static final double angleD = 0.00;
        public static final double angleFF = 0.0;

        public static final class Mod0{
            //Offset values to be updated
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.166504);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);

        }

        public static final class Mod1{
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset =  Rotation2d.fromRotations(	0.741943);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            
        }

        public static final class Mod2{
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.162109);
            public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            
        }

        public static final class Mod3{
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset =  Rotation2d.fromRotations(	0.016846);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID,canCoderID, angleOffset);
            
        }
    }
/*
    public static final class IntakeConstants{
        public static final int intakeMotorID;
        public static final double maxMotorVelocity;

        public static final int freeCurrentLimit;
        public static final int stallCurrentLimit;
    }

    public static final class LauncherConstants{
        public static final int topLaunchMotorID;
        public static final int bottomLauncherMotorID;

        public static final double maxMotorVelocityPercent;

        public static final int freeCurrentLimit;
        public static final int stallCurrentLimit;
    }

    public static final class ArmConstants{}

    public static final class AutoConstants{}
*/
}

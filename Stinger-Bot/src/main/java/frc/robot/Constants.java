package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
public class Constants {
    public static final class RobotConstants{
        public static final boolean isTuningMode = true;
        public static final boolean isShooterTuningMode = true;
    }

    public static final class ControllerConstants{
        public static final int driverControllerPort = 0;
        public static final int operatorControllerPort = 1;
        public static final double deadbandRange = 0.10;
    }

    public static final class Swerve{
        //Gyro ID
        public static final int pigeonID = 13;

        //DriveTrain Constants
        public static final double trackWidth = Units.inchesToMeters(20.75);
        public static final double wheelbase  = Units.inchesToMeters(20.75);
        public static final double whealDiameter = Units.inchesToMeters(4.00);
        public static final double wheelCircumference = whealDiameter * Math.PI;

        public static final double driveGearRatio = (6.75 / 1.0); //6.75:1
        public static final double angleGearRatio = (150.0/7.0) / 1.0; //150/7:1

        //Encoder converstion factors
        public static final double driveEncoderPositionFactor = wheelCircumference / driveGearRatio;
        //Rotation to meters 

        public static final double driveEncoderVelocityFactor = driveEncoderPositionFactor / 60;
        //RPM to meters per sec
        //public static final double integratedAngleEncoderPositionFactor = angleGearRatio * 360;
        //Rotations to degrees
        //public static final double integratedAngleEncoderVelocityFactor = integratedAngleEncoderPositionFactor / 60;
        //RPM to degrees per sec
        public static final double anglePositionFactor = (360 / angleGearRatio);//Degrees per shaft rotation

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(wheelbase/2, trackWidth/2),
            new Translation2d(wheelbase/2, -trackWidth/2),
            new Translation2d(-wheelbase/2, trackWidth/2),
            new Translation2d(-wheelbase/2, -trackWidth/2)

        );

        public static final double maxSpeed = 4.6; //meters per second
        public static final double maxAngleVelocity = 10.5; // radians per sec

        public static final int driveCurrentLimit = 40;//optimal limit based on NEO current limit data
        public static final int angleCurrentLimit = 40;

        public static final double voltageComp = 12.0;
 
        //feed forward values, need to be obtiaed though WPI charactarization tool
        public static final double driveKS = 0.667; // overcome friction
        public static final double driveKV = 2.44;//might need tune
        public static final double driveKA = 0.27;//might need tune


        //Drive Motor PID Values
        public static final double driveP = 0.00;
        public static final double driveI = 0.00;
        public static final double driveD = 0.00;
        
        //Angle Motor PID Values
        public static final double angleP = 0.0045;//0.0042
        public static final double angleI = 0.00;// always zero
        public static final double angleD = 0.00;

        public static final class Mod0{
            //Offset values to be updated
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(323.43);//tune
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);

        }

        public static final class Mod1{
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset =  Rotation2d.fromDegrees(30.67);//tune
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            
        }

        public static final class Mod2{
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(82.26); //tune
            public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            
        }

        public static final class Mod3{
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset =  Rotation2d.fromDegrees(211.90); //
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID,canCoderID, angleOffset);
            
        }
    }
    
    public static final class IntakeConstants{
        public static final int intakeID = 15;

        public static final int voltageComp = 12;
        
        public static final double intakeSpeed = 0.85;
        public static final double intakeSpitSpeed = -0.7;
    }


    public static final class TransferConstants{
        public static final int transferID = 16;
        
        //transfer
        public static final double transferSpeed = 1.0;
        public static final double tranSpitSpeed = -1.0;

        public static final double speedTolerance = 0.0;//to be determined
    }

    public static final class LauncherConstants{
        public static final int topLaunchID = 17;
        public static final int bottomLaunchID = 18;

        public static final double shooterTolerence = 5;

        //Duty cycle value [quick backup shots]
        public static final double dutySpeakerShot = 0.50;
        public static final double dutyAmpShot = 0.18;
    }

    public static final class ArmConstants{
        
        public static final int leadID = 19;
        public static final int followID = 20;
        public static final double kS = 0;//.91
        public static final double kG = 0;
        public static final double kA = 0;
        public static final double kP = 0;//40
        public static final double kI = 0;
        public static final double kD = 0;//.01
        public static final double kCruiseVelocity = Math.PI * 4;//tune
        public static final double kAcceleration = Math.PI;//tune
        public static final double kJerk = Math.PI * 80; //tune
        public static final boolean kClockwisePositive = false;//Lead talon is on the left [With Convention]
        public static final double kErrorTolerance = .03;//probably okay
        public static final double kSensorToMechanismGearRatio = 92.85;//Actual: 92.8571428571
        public static final int kCurrentLimit = 40;
    }

    public static final class ClimberConstants{}

    public static final class AutoConstants{}

    public static final class DIOConstants {
        public static final int encoderDioPort = 0;
        public static final int photoSensDioPort = 7;
    }

}

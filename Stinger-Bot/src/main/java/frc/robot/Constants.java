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

        public static final double driveGearRatio = (5.90 / 1.0); //6.75:1
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

        public static final double maxSpeed = 5.24; //meters per second
        public static final double maxAngleVelocity = 10.5; // radians per sec

        public static final int driveCurrentLimit = 40;//optimal limit based on NEO current limit data
        public static final int angleCurrentLimit = 20;

        public static final double voltageComp = 12.0;
 
        //feed forward values, need to be obtiaed though WPI charactarization tool
        public static final double driveKS = 0.667; // overcome friction
        public static final double driveKV = 2.44;//might need tune
        public static final double driveKA = 0.27;//might need tune

        public static final Boolean openLoopDrive = false;

        //Drive Motor PID Values
        /*
         * All values up until last day of closed loop drive remained zero  
         * last used closed loop drive on 4/12/2024
         * Now using open loop don't forget to map your joystick deadband
         * So that new deadband is now zero
         */
        public static final double driveP = 0.0;
        public static final double driveI = 0.00;
        public static final double driveD = 0.00;
        
        //Angle Motor PID Values
        public static final double angleP = 0.0070;//0.0045
        public static final double angleI = 0.00;// always zero
        public static final double angleD = 0.00;

        public static final class Mod0{
            //Offset values to be updated
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(323.52);//tune
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
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(82.41); //tune
            public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            
        }

        public static final class Mod3{
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset =  Rotation2d.fromDegrees(212.78); //
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID,canCoderID, angleOffset);
            
        }
    }
    
    public static final class IntakeConstants{
        public static final int intakeID = 15;

        public static final int voltageComp = 12;
        
        public static final double intakeSpeed = 1;
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

        public static final double shooterTolerence = 10;

        //Duty cycle value [quick backup shots]
        public static final double dutySpeakerShot = 0.50;
        public static final double dutyAmpShot = 0.22;//Origin .20
    }

    public static final class ArmConstants{
        
        public static final int leadID = 19;
        public static final int followID = 20;
        public static final double kS = .45;
        public static final double kV = 1.77;//1.35
        public static final double kP = 40;//39
        public static final double kI = 0;//0
        public static final double kD = .0005;//.001
        public static final double kCruiseVelocity = Math.PI * 4;
        public static final double kAcceleration = Math.PI*2.4;//2
        public static final double kJerk = Math.PI * 20; //Math.PI * 15
        public static final boolean kClockwisePositive = false;//Lead talon is on the left [With Convention]
        public static final double kErrorTolerance = .05;
        public static final double kSensorToMechanismGearRatio = 92.85;//Actual: 92.8571428571
        public static final int kCurrentLimit = 70;
    }

    public static final class ClimberConstants{
        //Units in rotaitons
        public static final double kP = 0.040;
        public static final double kI = 0;
        public static final double kD = 0;

        //moves climbers to max height using dutycycle and log output
        //use number slighly under max for tolerance
        public static final double maxHight = 480;
        //min height should be slightly higher than 0 to account for slippage and such
        public static final double minHeight = 19.6;
    }

    public static final class AutoConstants{}

    public static final class DIOConstants {
        public static final int encoderDioPort = 0;
        public static final int photoSensDioPort = 7;
    }

}

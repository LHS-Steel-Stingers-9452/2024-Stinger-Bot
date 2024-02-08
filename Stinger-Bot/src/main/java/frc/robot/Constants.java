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
        public static final double SLEW_RATE = 4;
    }

    public static final class Swerve{
        //Gyro ID
        public static final int PIGEON_ID = 13;

        public static final boolean invertGyro = false;

        //DriveTrain Constants
        //ready
        public static final double trackWidth = Units.inchesToMeters(24.75);
        public static final double wheelbase  = Units.inchesToMeters(24.75);
        public static final double whealDiameter = Units.inchesToMeters(4.00);
        public static final double wheelCircumference = whealDiameter * Math.PI;

        //ready
        public static final double driveGearRatio = 6.75 / 1.0; //6.75:1
        public static final double angleGearRatio = (150.0/7.0) / 1.0; //150/7:1

        //Encoder converstion factors
        public static final double driveEncoderPositionFactor = driveGearRatio * wheelCircumference; 
        //Rotations to meters [limear distnce in meters]
        public static final double driveEncoderVelocityFactor = driveEncoderPositionFactor / 60;
        //RPM to meters per sec
        public static final double integratedAngleEncoderPositionFactor = angleGearRatio * 2 * Math.PI;
        //Rotations to radians
        public static final double integratedAngleEncoderVelocityFactor = integratedAngleEncoderPositionFactor / 60;
        //RPM to radian per sec
        public static final double anglePositionFactor = 2*Math.PI / angleGearRatio;//Radians per shaft rotation

        //ready
        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(wheelbase/2, trackWidth/2),
            new Translation2d(wheelbase/2, -trackWidth/2),
            new Translation2d(-wheelbase/2, trackWidth/2),
            new Translation2d(-wheelbase/2, -trackWidth/2)

        );
        //meters/sec
        //ready
        public static final double maxSpeed = 4.0; //values are subject to change upon testing 
        public static final double maxAngleVelocity = 4.0; 

        public static final int driveCurrentLimit = 70;//subject to change
        public static final int angleCurrentLimit = 30;

        public static final boolean openLoop = true; // toggle by preference default at true

        public static final double voltageComp = 12.0; // from template
 
        //feed forward values, need to be obtiaed though WPI charactarization tool
        public static final double driveKS = 0.667; // from template
        public static final double driveKV = 2.44;
        public static final double driveKA = 0.27;


        //Drive Motor PID Values
        public static final double driveP = 0.00;
        public static final double driveI = 0.0;
        public static final double driveD = 0.0;
        public static final double driveFF = 0.0;

        //Angle Motor PID Values
        public static final double angleP = 0.00;
        public static final double angleI = 0.00;
        public static final double angleD = 0.00;
        public static final double angleFF = 0.00;

        public static final class Mod0{
            //Offset values to be updated
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(109.160);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);

        }

        public static final class Mod1{
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset =  Rotation2d.fromDegrees(267.62);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            
        }

        public static final class Mod2{
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(173.75);
            public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            
        }

        public static final class Mod3{
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset =  Rotation2d.fromDegrees(-5.44);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID,canCoderID, angleOffset);
            
        }
    }
/*
    public static final class AutoConstants{}
*/
}

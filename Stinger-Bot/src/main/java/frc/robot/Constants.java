package frc.robot;

import frc.robot.SwerveModuleConstants;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
public class Constants {
    public final static class ControllerConstants{
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final double DEADBANDRANGE = .3;

        //public static final SlewRateLimiter translationFilter = new SlewRateLimiter(3);
        public static final double SLEW_RATE = 3;
    }

    public static final class Swerve{
        //Gyro ID
        public static final int PIGEON_ID = 12;

        //DriveTrain Constants
        public static final double TRACK_WIDTH = Units.inchesToMeters(24.75);
        public static final double WHEELBASE  = Units.inchesToMeters(24.75);
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.00);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        public static final double DRIVE_GEAR_RATIO = 6.75/1.0; //6.75:1
        public static final double ANGLE_GEAR_RATIO = (150.0/7.0)/1.0; //150/7:1

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEELBASE/2, TRACK_WIDTH/2),
            new Translation2d(WHEELBASE/2, -TRACK_WIDTH/2),
            new Translation2d(-WHEELBASE/2, TRACK_WIDTH/2),
            new Translation2d(-WHEELBASE/2, -TRACK_WIDTH/2)

        );



        public static final class Mod0{
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 8;
            public static final Rotation2d angeOffset;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);

        }

        public static final class Mod1{
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 9;
            public static final Rotation2d angeOffset;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            
        }

        public static final class Mod2{
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 10;
            public static final Rotation2d angeOffset;
            public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            
        }

        public static final class Mod3{
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 11;
            public static final Rotation2d angeOffset;
            public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, angleOffset);
            
        }
    }
    public static final class AutoConstants{

    }
    
}

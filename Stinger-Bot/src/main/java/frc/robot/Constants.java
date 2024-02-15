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
        public static final double DEADBANDRANGE = 0.15;
        public static final double SLEW_RATE = 4;
    }

    public static final class Swerve{
        //Gyro ID
        public static final int pigeonID = 13;

        //DriveTrain Constants
        public static final double trackWidth = Units.inchesToMeters(24.75);
        public static final double wheelbase  = Units.inchesToMeters(24.75);
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

        public static final double maxSpeed = 4.5; //values are subject to change upon testing 
        public static final double maxAngleVelocity = 4.5; // calculate actual max speed/velocity

        public static final int driveCurrentLimit = 40;//optimal limit based on NEO current limit data
        public static final int angleCurrentLimit = 30;

        public static final double voltageComp = 12.0;
 
        //feed forward values, need to be obtiaed though WPI charactarization tool
        public static final double driveKS = 0.667; // overcome friction
        public static final double driveKV = 2.44;
        public static final double driveKA = 0.27;


        //Drive Motor PID Values
        public static final double driveP = 0.00;
        public static final double driveI = 0.00;
        public static final double driveD = 0.00;
        
        //Angle Motor PID Values
        public static final double angleP = 0.0026;
        public static final double angleI = 0.00;
        public static final double angleD = 0.00;

        public static final class Mod0{
            //Offset values to be updated
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(323.61);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);

        }

        public static final class Mod1{
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset =  Rotation2d.fromDegrees(31.02);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            
        }

        public static final class Mod2{
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(301.37);
            public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            
        }

        public static final class Mod3{
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset =  Rotation2d.fromDegrees(350.41);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID,canCoderID, angleOffset);
            
        }
    }
    public static final class Intake{
        public static final int intakeID = 15;
        public static final int intakeCurrentLimit = 0;
    }

    public static final class Transfer{
        public static final int transferID = 16;

        public static final int transferCurrentLimit = 0;
    }

    public static final class Launcher{
        public static final int leftLaunchID = 17;
        public static final int rightLaunchID = 18;

        public static final int launcherP = 0;
        public static final int launcherI = 0;
        public static final int launcherD = 0;
    }

    public static final class Arm{
        public static final int leftArmID = 19;
        public static final int rightArmID = 20;

        public static final double armGearRatio;

        public static final double armEncoderPositionFactor = 360;
        //Rotation to degrees [not accounting for gearRatio]

       public static final double driveEncoderVelocityFactor;
        //RPM to ___

        public static final int armP = 0;
        public static final int armI = 0;
        public static final int armD = 0;

        public static final double lowPosition;
        public static final double midPosition;
        public static final double highPosition;
    }

    public static final class Climber{
        public static final int leftClimbID = 21;
        public static final int rightClimbID = 22;

        public static final int climberCurrentLimit = 0;
    }

    public static final class AutoConstants{}

}

package frc.robot;

//import frc.robot.SwerveModuleConstants;

//import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
//import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Util.Setpoints;
import frc.robot.Util.Setpoints.GameState;
public class Constants {
    public static final class RobotConstants{
        public static final boolean isTuningMode = true;//in tunning mode
        public static final boolean isDrivingTuningMode = false;//need
        public static final boolean isArmTurningMode = true;//in process
        public static final boolean isIntakeTuningMode = false;//done
        public static final boolean isTransferTuningMode = false;//done
        public static final boolean isShooterTuningMode = false;//need

        /**Shooter and Arm Setpoints */
        public static final Setpoints STOWED = new Setpoints(2, 0.4, 0, 0, GameState.STOWED);//1 degree with 2 tolerance
        public static final Setpoints INTAKE = new Setpoints(1, 0.4, 0, 0, GameState.STOWED);//1 degree with 2 tolerance

        public static final Setpoints SPEAKER = new Setpoints(1, 1, 30, 30, GameState.SPEAKER);//1 degree with 30RPS

        //amp will be close to horizontal position
        public static final Setpoints AMP = new Setpoints(88, 0.4, 20, 20, GameState.AMP);//88deg, .4 tolerance, 20RPS



        public static final Setpoints PODIUM = new Setpoints(0, 0, 0, 0, GameState.PODIUM);
        public static final Setpoints WING = new Setpoints(0, 0, 0, 0, GameState.WING);

        //public static final Setpoints TRAP = new Setpoints(0, 0, 0, 0, GameState.TRAP);
    }

    public static final class ControllerConstants{
        public static final int driverControllerPort = 0;
        public static final int operatorControllerPort = 1;
        public static final double deadbandRange = 0.15;
        public static final double slewRate = 4;
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
        public static final double driveKV = 2.44;//might need tune
        public static final double driveKA = 0.27;//might need tune


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
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(323.61);//tune
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);

        }

        public static final class Mod1{
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset =  Rotation2d.fromDegrees(31.02);//tune
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            
        }

        public static final class Mod2{
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(301.37); //tune
            public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            
        }

        public static final class Mod3{
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset =  Rotation2d.fromDegrees(350.41); //tune
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID,canCoderID, angleOffset);
            
        }
    }
    
    public static final class IntakeConstants{
        public static final int intakeID = 15;

        public static final int voltageComp = 12;
        
        public static final double intakeSpeed = .5;
        public static final double intakeSpitSpeed = -.5;
    }


    public static final class TransferConstants{
        public static final int transferID = 16;
        
        //transfer
        public static final double transferSeed = 0.25;
        public static final double tranSpitSpeed = -0.25;

        public static final double speedTolerance = 0.0;//to be determined
    }

    public static final class LauncherConstants{
        public static final int leftMotorID = 19;
        public static final int rightMotorID = 20;

        //Duty cycle
        public static final double intakeFromShooterSpeed = -.20;

        public static final double shooterTolerence = 10.0;//to be determined
    }

    public static final class ArmConstants{
        /*
        * Calibrating the Arm Angle
        * 
        * - Turn the robot off and push the Arm against its hard stop in the STOWED position <br>
        * - Turn the robot on and connect to the robot (Do not enable) <br>
        * - Open Shuffleboard and find the box with the value for "Arm Angle Uncorrected" <br>
        * - Copy this value into the constant named kARM_STARTING_OFFSET in the "ArmConstants" section of Constants.java <br>
        * - The value should be > 0.0 (i.e. not negative). If it is 0.0 or less, then there is an encoder issue.
        * - The value should be between 30-120 degrees. Anything over 200 likely means the encoder zero point is not in the right spot)<br>
        * - You want to make sure the value you choose is just slightly smaller than the lowest number that appears in "Arm Angle Uncorrected".
        * - Otherwise you may get negative readings for the Arm Current Angle, and error checking may prevent the Arm motors from moving.
        * - Move the Arm to the horizontal position and again check the value in the "Arm Angle Uncorrected" box. <br>
        * - Copy this value into the constant named kARM_HORIZONTAL_OFFSET. (It should be between 90-160 degrees).<br>
        * - Save the file and deploy code to the robot. Make sure the Arm starts in the STOWED position. <br>
        * - If the value for Arm Current Angle is a negative value do not enable, and try to do the offsets again <br>
        * - If it is still negative, then there is an issue with the encoder. <br>
        */
        public static final int leadID = 27;
        public static final int followID = 28;

        //measured when the arm is in the STOWED position
        public static final double armStartingOffset = 0.0;//TBD
        //measured when the arm is horizontal
        public static final double armHorizontalOffset = 0.0;//TBD

        //feedforward gains
        //tbd current values were obtained through means of an online calculator
        public static final double armKS = 0; // The Static Gain, in volts
        public static final double armKG = 0.40; // The Gravity Gain, in volts
        public static final double armKV = 1.3; // The Velocity Gain, in volt seconds per radian
        public static final double armKA = 0.02; // The acceleration gain, in volt seconds^2 per radian

        //PID Control gains in volts
        public static final int armP = 0;
        public static final int armI = 0;
        public static final int armD = 0;

        //Profiled PID constants
        public static final double armCruise = 4.00; // Radians per second
        public static final double armAcceleration = 10.00; // Radians per second^2

        public static final double encoderDutyCycleMin = 1.0/1025.0; //~0
        public static final double encoderDutyCycleMax = 1024.0/1025.0; //~1

        // Motor Neutral dead-band : Range 0.001 -> 0.25
        public static final double neutralDeadBand = 0.005; //-> 1.25
    }

    public static final class ClimberConstants{
        public static final int leftClimbID = 0;
        public static final int rightClimbID = 0;

        public static final int climberCurrentLimit = 0;
    }

    public static final class AutoConstants{}

    public static final class DIOConstants {
        public static final int encoderDioPort = 0;
        public static final int photoSensDioPort = 1;
    }

}

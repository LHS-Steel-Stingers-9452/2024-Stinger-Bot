// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

//import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.SwerveModuleConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;

/** Add your docs here. */
public class SwerveModule {
    public int moduleNumber;
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    //motors
    private final CANSparkMax driveMotor;
    private final CANSparkMax angleMotor;

    //integrated encoders
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder integratedAngleEncoder;

    //absolute encoder
    private final CANcoder canCoder;
    private CANcoderConfigurator canCoderConfigurator;
    private CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
    
    //PID controllers
    private final SparkPIDController drivePIDController;
    private final SparkPIDController anglePIDController;


    //feed forward: used for closed-loop
    private final SimpleMotorFeedforward driveFeedforward = 
        new SimpleMotorFeedforward(Swerve.driveKS, Swerve.driveKV, Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants swerveConstants){
        //initialize variables here
        this.moduleNumber = moduleNumber;

        driveMotor = new CANSparkMax(swerveConstants.driveMotorCanID, MotorType.kBrushless);
        angleMotor = new CANSparkMax(swerveConstants.angleMotorCanID, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        integratedAngleEncoder = angleMotor.getEncoder();

        canCoder = new CANcoder(swerveConstants.canCoderID);
        angleOffset = swerveConstants.canCoderOffset;

        drivePIDController = driveMotor.getPIDController();
        anglePIDController = angleMotor.getPIDController();
        
        driveConfig();
        angleMotorConfig();
        angleEncoderConfig();

        lastAngle = getState().angle;

    }
    //ready units are now meters and radians 
    public void driveConfig() {
        driveMotor.restoreFactoryDefaults();
        //driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20);
        //driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20);
        //driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 50);

        driveMotor.setInverted(false);
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        driveMotor.setSmartCurrentLimit(Swerve.driveCurrentLimit);
        driveMotor.enableVoltageCompensation(Swerve.voltageComp);//usefull for consitency, treats as if battery were always at 12 volts

        driveEncoder.setPositionConversionFactor(Swerve.driveEncoderPositionFactor);// distance in meters
        driveEncoder.setVelocityConversionFactor(Swerve.driveEncoderVelocityFactor);//meters per sec

        //only use P value
        drivePIDController.setP(Swerve.driveP);
        drivePIDController.setI(Swerve.driveI);
        drivePIDController.setD(Swerve.driveD);

        driveMotor.burnFlash();
        driveEncoder.setPosition(0.0);

        
    }
    // ready untis are now meters and radians
    public void angleMotorConfig(){
        angleMotor.restoreFactoryDefaults();
        //angleMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 500);
        //angleMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20);
        //angleMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 500);

        angleMotor.setInverted(true);
        angleMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        angleMotor.setSmartCurrentLimit(Swerve.angleCurrentLimit);
        angleMotor.enableVoltageCompensation(Swerve.voltageComp);

        integratedAngleEncoder.setPositionConversionFactor(Swerve.anglePositionFactor);//Degrees per shaft rotation

       anglePIDController.setP(Swerve.angleP);
       anglePIDController.setI(Swerve.angleI);
       anglePIDController.setD(Swerve.angleD);
        angleMotor.burnFlash();

        resetToAbsolute();
    }

    public void angleEncoderConfig(){
        canCoderConfigurator = canCoder.getConfigurator();
        canCoderConfigurator.apply(canCoderConfiguration);
        //restores factory defaults

        canCoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        canCoderConfigurator.apply(canCoderConfiguration);

        canCoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        canCoderConfigurator.apply(canCoderConfiguration);
     
        /* 
        canCoderConfiguration.MagnetSensor.MagnetOffset = (angleOffset.getRotations());
        canCoderConfigurator.apply(canCoderConfiguration);
        */
    
    }

    public void resetToAbsolute(){
        //degrees units to match anglePositionConversionFactor
        double angelAbsolutePosition = getCanCoderValue().getDegrees() - angleOffset.getDegrees();
        integratedAngleEncoder.setPosition(angelAbsolutePosition);//integrated ecnoder is reset and given canCoder value; absolute position
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    } 

    public Rotation2d getCanCoderValue(){
        return Rotation2d.fromRotations(canCoder.getAbsolutePosition().getValue());
        //Rotation2D.fromRotations() will automaticly convert from [0, 1) to [0, 360) when needed at the request of getDegrees()
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            driveEncoder.getPosition(),
            getAngle()
        );
    }

    public void setDesiredState(SwerveModuleState desiredModuleState){
        /* 
        SmartDashboard.putNumber("Optimized " + moduleNumber + " Speed Setpoint: ", desiredState.speedMetersPerSecond);
        SmartDashboard.putNumber("Optimized " + moduleNumber + " Angle Setpoint(degrees): ", desiredState.angle.getDegrees());
        */
        desiredModuleState = CustomModuleState.optimize(desiredModuleState, getState().angle);

        setAngle(desiredModuleState);
        setSpeed(desiredModuleState);
    }

    private void setSpeed(SwerveModuleState desiredModuleState){
        //only run closed loop
        
        double velocity = desiredModuleState.speedMetersPerSecond;
        drivePIDController.setReference(velocity, ControlType.kVelocity, 0, driveFeedforward.calculate(velocity));
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle =
            (Math.abs(desiredState.speedMetersPerSecond) <= (Swerve.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle;
        //PID wrapping MathUtil.inputModulus(angle.getRadians(), -Math.PI, Math.PI)
        anglePIDController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
        }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.SwerveModuleConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

//import edu.wpi.first.units.Mult;

//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
//import com.ctre.phoenix6.configs.MagnetSensorConfigs;

//import edu.wpi.first.math.util.Units;


/** Add your docs here. */
public class SwerveModule {
    public int moduleNumber;

    //encoder/module configuration values(on top because they are physical factors)
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    //motors
    private final CANSparkMax driveMotor;
    private final CANSparkMax angleMotor;

    //encoders
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder integratedAngleEncoder;
    private final CANcoder canCoder;
    private CANcoderConfigurator canCoderConfigurator;
    private CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
    
    //controllers
    private final SparkPIDController drivePIDController;
    private final SparkPIDController anglePIDController;

/* 
    //feed forward WEIRD: use for close-loop
    private final SimpleMotorFeedforward driveFeedforward = 
        new SimpleMotorFeedforward(Swerve.driveKS, Swerve.driveKV, Swerve.driveKA);
*/
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
        
        //calling swerve module config functions
        driveConfig();
        angleMotorConfig();
        angleEncoderConfig();

        //obtains last angle based on module state
        lastAngle = getState().angle;

    }

    public void driveConfig() {
        driveMotor.restoreFactoryDefaults();
        //only update can bus usage if effecting performance
        driveMotor.setInverted(false);
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        driveMotor.setSmartCurrentLimit(Swerve.driveCurrentLimit);
        //Conversion factors and continous PID wrapping
        driveEncoder.setPositionConversionFactor(Swerve.DRIVE_ENCODER_POSITION_FACTOR);
        driveEncoder.setVelocityConversionFactor(Swerve.DRIVE_ENCODER_VELOCITY_FACTOR);
        drivePIDController.setPositionPIDWrappingEnabled(true);
        drivePIDController.setPositionPIDWrappingMinInput(-180);
        drivePIDController.setPositionPIDWrappingMaxInput(180);
        drivePIDController.setP(Swerve.driveP);
        drivePIDController.setI(Swerve.driveI);
        drivePIDController.setD(Swerve.driveD);
        drivePIDController.setFF(Swerve.driveFF);
        //setFeedforward  use for closed loop
        //voltage compensation?
        driveMotor.burnFlash();
        //Reset As A Saftery Measure
        driveEncoder.setPosition(0.0);

        
    }

    public void angleMotorConfig(){
        angleMotor.restoreFactoryDefaults();
        //only update can bus usage if effecting performance
        angleMotor.setInverted(true);
        angleMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        angleMotor.setSmartCurrentLimit(Swerve.angleCurrentLimit);
        //Converstion factors and continous PID wrapping
        integratedAngleEncoder.setPositionConversionFactor(Swerve.INTEGRATED_ANGLE_ENCODER_POSITION_FACTOR);
        integratedAngleEncoder.setVelocityConversionFactor(Swerve.INTEGRATED_ANGLE_ENCODER_VELOCITY_FACTOR);
        anglePIDController.setPositionPIDWrappingEnabled(true);
        anglePIDController.setPositionPIDWrappingMinInput(-180);
        anglePIDController.setPositionPIDWrappingMaxInput(180);
       anglePIDController.setP(Swerve.angleP);
       anglePIDController.setI(Swerve.angleI);
       anglePIDController.setD(Swerve.angleD);
       anglePIDController.setFF(Swerve.angleFF);
        //setFeedforward use for closed loop
        //voltage compensation?
        angleMotor.burnFlash();
        //Reset As A Saftery Measure
        resetToAbsolute();
    }

    public void angleEncoderConfig(){
        //canCoder config practice
        canCoderConfigurator = canCoder.getConfigurator();
        canCoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        canCoderConfigurator.apply(canCoderConfiguration);

        canCoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        canCoderConfigurator.apply(canCoderConfiguration);

        canCoderConfiguration.MagnetSensor.MagnetOffset = angleOffset.getRotations();
        canCoderConfigurator.apply(canCoderConfiguration);
    }

    private void resetToAbsolute(){
        double angelAbsolutePosition = getCanCoderValue().getDegrees() - angleOffset.getDegrees();
        integratedAngleEncoder.setPosition(angelAbsolutePosition);//integrated ecnoder is reset and given canCoder value
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    } 

    public Rotation2d getCanCoderValue(){
        return Rotation2d.fromRotations(canCoder.getAbsolutePosition().getValue());
        
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            /* 
            Use if rotation units don't work
            Converts from Rotations to Distance traveled
            driveEncoder.getPosition() * (Swerve.WHEEL_DIAMETER * Math.PI),
            */
            driveEncoder.getPosition(),
            getAngle()
        );
    }

    //If necessary add closed loop option
    public void setDesiredState(SwerveModuleState desiredModuleState){
        SwerveModuleState desiredState = new SwerveModuleState(desiredModuleState.speedMetersPerSecond, getState().angle);
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        driveMotor.set(desiredState.speedMetersPerSecond / Swerve.maxDriveSpeed);
        // Prevent rotation when jittering is less than 1%
        Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Swerve.maxAngleVelocity * 0.01))
        ? lastAngle
        : desiredState.angle;
        anglePIDController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }
}

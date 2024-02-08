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
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;

//import edu.wpi.first.units.Mult;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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


    //feed forward WEIRD: use for close-loop
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
        driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20);
        driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20);
        driveMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 50);
        driveMotor.setInverted(false);
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        driveMotor.setSmartCurrentLimit(Swerve.driveCurrentLimit);
        driveMotor.enableVoltageCompensation(Swerve.voltageComp);
        driveEncoder.setPositionConversionFactor(Swerve.DRIVE_ENCODER_POSITION_FACTOR);
        driveEncoder.setVelocityConversionFactor(Swerve.DRIVE_ENCODER_VELOCITY_FACTOR);
        drivePIDController.setPositionPIDWrappingEnabled(true);
        drivePIDController.setPositionPIDWrappingMinInput(-180);
        drivePIDController.setPositionPIDWrappingMaxInput(180);
        drivePIDController.setP(Swerve.driveP);
        drivePIDController.setI(Swerve.driveI);
        drivePIDController.setD(Swerve.driveD);
        drivePIDController.setFF(Swerve.driveFF);
        driveMotor.burnFlash();
        //Reset As A Saftery Measure
        driveEncoder.setPosition(0.0);

        
    }

    public void angleMotorConfig(){
        angleMotor.restoreFactoryDefaults();
        //only update can bus usage if effecting performance
        angleMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 500);
        angleMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20);
        angleMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 500);
        angleMotor.setInverted(true);
        angleMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        angleMotor.setSmartCurrentLimit(Swerve.angleCurrentLimit);
        angleMotor.enableVoltageCompensation(Swerve.voltageComp);
        integratedAngleEncoder.setPositionConversionFactor(Swerve.ANGLE_POSITION_FACTOR);
        //integratedAngleEncoder.setVelocityConversionFactor(Swerve.INTEGRATED_ANGLE_ENCODER_VELOCITY_FACTOR);
        anglePIDController.setPositionPIDWrappingEnabled(true);
        anglePIDController.setPositionPIDWrappingMinInput(-180);
        anglePIDController.setPositionPIDWrappingMaxInput(180);
       anglePIDController.setP(Swerve.angleP);
       anglePIDController.setI(Swerve.angleI);
       anglePIDController.setD(Swerve.angleD);
       anglePIDController.setFF(Swerve.angleFF);
        angleMotor.burnFlash();

        resetToAbsolute();
    }

    //unsure
    public void angleEncoderConfig(){
        //canCoder config practice
        canCoderConfigurator = canCoder.getConfigurator();
        //Multiply range by 360 to convert to degrees
        canCoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        canCoderConfigurator.apply(canCoderConfiguration);

        canCoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        canCoderConfigurator.apply(canCoderConfiguration);
        /*
         * Maps angle offsets from degrees to a domain of [-1,1] as supported by the CAN coder
         */
        canCoderConfiguration.MagnetSensor.MagnetOffset = ((angleOffset.getDegrees()/180.00) - 1); 
        canCoderConfigurator.apply(canCoderConfiguration);

    }

    public void resetToAbsolute(){
        double angelAbsolutePosition = getCanCoderValue().getRotations() - angleOffset.getRotations();
        integratedAngleEncoder.setPosition(angelAbsolutePosition);//integrated ecnoder is reset and given canCoder value; absolute position
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    } 

    public Rotation2d getCanCoderValue(){
        //multiply by 360 to go from a domain of [0, 1) to [0 to 360). This turns the absolute position into degrees
        return Rotation2d.fromDegrees(canCoder.getAbsolutePosition().getValue() * 360);
        
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());

    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }

    public void setDesiredState(SwerveModuleState desiredModuleState, boolean isOpenLoop){

        SwerveModuleState desiredState = new SwerveModuleState(desiredModuleState.speedMetersPerSecond, getState().angle);

        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        SmartDashboard.putNumber("Optimized " + moduleNumber + " Speed Setpoint: ", desiredState.speedMetersPerSecond);
        SmartDashboard.putNumber("Optimized " + moduleNumber + " Angle Setpoint(degrees): ", desiredState.angle.getDegrees());

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    
    private void setSpeed(SwerveModuleState desiredModuleState, boolean isOpenLoop){
        if (isOpenLoop){
            //calculates percent output
            driveMotor.set(desiredModuleState.speedMetersPerSecond / Swerve.maxSpeed);
        } else {
            drivePIDController.setReference(
                desiredModuleState.speedMetersPerSecond, 
                ControlType.kVelocity,
                0,
                driveFeedforward.calculate(desiredModuleState.speedMetersPerSecond)
                );
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        //Is this why turn motors move initially upon deployment?
        //Only place that make turn motors move
        Rotation2d angle =
            (Math.abs(desiredState.speedMetersPerSecond) <= (Swerve.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle;

        anglePIDController.setReference(angle.getRotations(), ControlType.kPosition);

        /* 
        anglePIDController.setReference(angle.getRotations(), 
        ControlType.kPosition, 
        0, 
        //omega to radians/sec                              Module KV: (maxVolts) / ((degreesPerRotation) * (maxMotorSpeedRPM / gearRatio) * (minutesPerSecond)
        Math.toDegrees((desiredState.angle.getRadians()/60)) * (Units.rotationsToDegrees((((5676 * 7) / 372)) / 60)));
        */
        lastAngle = angle;
        }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.SwerveModuleConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix6.hardware.CANcoder;
//import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

/** Add your docs here. */
public class SwerveModule {
    private final int moduleNumber;

    //encoder/module configuration values(on top because they are physical factors)
    private double lastAngle;
    private Rotation2d canCoderOffset;

    //motors
    private final CANSparkMax driveMotor;
    private final CANSparkMax angleMotor;

    //encoders
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder integratedAngleEncoder;
    private final CANcoder canCoder;

    //controllers
    private final SparkPIDController drivePIDController;
    private final SparkPIDController anglePIDController;

    //feed forward WEIRD
    private final SimpleMotorFeedforward driveFeedforward;

    public SwerveModule(int moduleNumber, SwerveModuleConstants swerveConstants){
        //initialize variables here
        this.moduleNumber = moduleNumber;

        driveMotor = new CANSparkMax(swerveConstants.driveMotorCanID, MotorType.kBrushless);
        angleMotor = new CANSparkMax(swerveConstants.angleMotorCanID, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        integratedAngleEncoder = angleMotor.getEncoder();
        canCoder = new CANcoder(swerveConstants.canCoderID);
        canCoderOffset = swerveConstants.canCoderOffset;

        drivePIDController = driveMotor.getPIDController();
        anglePIDController = angleMotor.getPIDController();

        driveFeedforward = new SimpleMotorFeedforward(moduleNumber, moduleNumber, moduleNumber);

        configureDrive();
        configureAngle();
        //lastAngle = getState().angle.getradians()


    }

    public void configureDrive() {
        //configure motors
        driveMotor.setInverted(false);
        //confiugre encoders
        //configure other stuff
    }

    public void configureAngle(){
        angleMotor.setInverted(false);
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;

import com.ctre.phoenix6.hardware.TalonFX;


public class Shooter extends SubsystemBase {
  /** Creates a new Launcher. */
  private final TalonFX leftMotor = new TalonFX(LauncherConstants.leftMotorID);
  private final TalonFX rightMotor = new TalonFX(LauncherConstants.rightMotorID);

  public Shooter(){

  }

}

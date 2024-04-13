// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Util.LimelightHelpers;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */

  /* Limelight Classes
   * LimelightHelpers.PoseEstimate
   * LimelightHelpers.LimelightTarget_Retro
   * LimelightHelpers.LimelightTarget_Fiducial
   * LimelightHelpers.LimelightTarget_Barcode
   * LimelightHelpers.LimelightTarget_Classifier
   * LimelightHelpers.LimelightTarget_Detector
   * LimelightHelpers.Results
   * LimelightHelpers.LimelightResults
   * (Pure Static) LimelightHelpers
   */

  LimelightHelpers.LimelightResults llResults = LimelightHelpers.getLatestResults("");
  
  public Limelight() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
}

}

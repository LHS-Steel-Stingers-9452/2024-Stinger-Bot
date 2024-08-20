// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;


public class LimeLight extends SubsystemBase {
  /** Creates a new Limelight camera. */

  public void setRobotOrientation(double robotHeading){
    LimelightHelpers.SetRobotOrientation("Stinger_Cam", robotHeading, 0, 0, 0, 0, 0);
  }

  //rejection logic goes here, this includes filtering tags and such
  //Note you could use swtich case here
  public boolean rejectPoseEstimate(boolean rotInLimit){
    boolean rejectEstimate = false;
    if (rotInLimit || getPoseEstimate().tagCount == 0){
        rejectEstimate = true;
    }
    return rejectEstimate;
  }

  public PoseEstimate getPoseEstimate(){
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("Stinger_Cam");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

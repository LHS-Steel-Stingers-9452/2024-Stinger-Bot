package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
public class SwerveModuleConstants {
    public final int driveMotorCanID;
    public final int angleMotorCanID;
    public final int canCoderID;
    public final Rotation2d canCoderOffset;

    public SwerveModuleConstants(int driveMotorCanID, int angleMotorCanID, int canCoderID, Rotation2d angleOffset) {
        this.driveMotorCanID = driveMotorCanID;
        this.angleMotorCanID = angleMotorCanID;
        this.canCoderID = canCoderID;
        this.canCoderOffset = angleOffset;
    }
    
}

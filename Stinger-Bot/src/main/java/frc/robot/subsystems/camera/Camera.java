package frc.robot.subsystems.camera;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Camera extends TimedRobot {

    public void robotInit() {
        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setResolution(320, 240);
        camera.setFPS(60);

        ShuffleboardTab tab = Shuffleboard.getTab("Camera Feed");
        tab.add(camera);
    }
    
}

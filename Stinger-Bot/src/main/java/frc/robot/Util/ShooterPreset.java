package frc.robot.Util;

public class ShooterPreset {
    private double distance;
    private double armAngle;
    private double shooterVeloc;

    public ShooterPreset(double distance, double armAngle, double shooterVeloc){
        this.distance = distance;
        this.armAngle = armAngle;
        this.shooterVeloc = shooterVeloc;
    }

    //SECTION - getters
    public double getDistance(){
        return distance;
    }
    
    public double getArmAngle(){
        return armAngle;
    }

    public double getShooterVeloc(){
        return shooterVeloc;
    }

    //SECTION -  setters()
    public void seDistance(double newDistance){
        this.distance = newDistance;
    }

    public void setArmAngle(double newAngle){
        this.armAngle = newAngle;
    }

    public void setShooterVeloc(double newSpeed){
        this.shooterVeloc = newSpeed;
    }
    
}

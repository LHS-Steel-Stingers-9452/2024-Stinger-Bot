package frc.robot.Util;

public class ShooterPreset {
    private double armAngle;
    private double shooterVeloc;

    public ShooterPreset(double armAngle, double shooterVeloc){
        this.armAngle = armAngle;
        this.shooterVeloc = shooterVeloc;
    }

    //SECTION - getters
    public double getArmAngle(){
        return armAngle;
    }

    public double getShooterVeloc(){
        return shooterVeloc;
    }

    //SECTION -  setters()
    public void setArmAngle(double newAngle){
        this.armAngle = newAngle;
    }

    public void setShooterVeloc(double newSpeed){
        this.shooterVeloc = newSpeed;
    }
    
}

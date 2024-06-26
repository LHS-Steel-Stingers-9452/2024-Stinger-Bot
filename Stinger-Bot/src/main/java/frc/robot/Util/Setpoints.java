package frc.robot.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A Class to hold setpoints for both the Shooter and  Arm, since they work in tandem.
 * 
 * @param arm The Arm setpoint
 * @param shooter The Shooter setpoint
 * @param state The GameState these setpoints define
 */

public class Setpoints {
    public double arm;
    public double tolerance;
    public double shooter;
    public GameState state;

    /**
     * 
     * @param arm - value should be given in deggrees
     * @param tolerance 
     * @param shooter - value should be fed in RPM
     * @param state
     */
    public Setpoints(double arm, double tolerance, double shooter, GameState state){
        this.arm = arm;
        this.tolerance = tolerance;
        this.shooter = shooter;
        this.state = state;
    }

    public enum GameState{
        DEFAULT, SPEAKER, AMP, PODIUM, WING, TRAP
    }

    //Display the commanded ARM stte on the dashboard
    public static void displayArmState(GameState state){
        switch (state) {
            case DEFAULT:
                SmartDashboard.putString("Arm State", "STOWED");
                break;
            case SPEAKER:
                SmartDashboard.putString("Arm State", "SPEAKER");
                break;
            case AMP:
                SmartDashboard.putString("Arm State", "AMP");
                break;
            case PODIUM:
                SmartDashboard.putString("Arm State", "PODIUM");
                break;
            case WING:
                SmartDashboard.putString("Arm State", "WING");
                break;
            case TRAP:
                SmartDashboard.putString("Arm State", "TRAP");
                break; 
            default:
                SmartDashboard.putString("Arm State", "OTHER");
        }
    }
    
    
}
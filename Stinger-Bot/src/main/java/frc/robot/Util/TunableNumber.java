package frc.robot.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.RobotConstants;

/**
 * Class for a tunable number. 
 * Gets value from dashbaord in tuning mode, returns default value if not or value not in dashboard.
 */

public class TunableNumber {
    private static final String tableKey = "TunableNumbers";

    private String key;
    private double defaultValue;
    private boolean hasDefault = false;


    /**
     * Create a TunableNumber
     * 
     * @param dashboardKey Key on dashboard
     */
    public TunableNumber(String dashboardKey){
        this.key = tableKey + "/" + dashboardKey;
    }

    /**
     * Creates a new TunableNumber with the default value
     * 
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default Value
    */
    public TunableNumber(String dashboardKey, double defaultValue){
        this(dashboardKey);
        setDefault(defaultValue);
    }

    /**
     * Set the default value of the number. The default value can only be set once.
     * 
     * 
     * @param defaultValue The default value
     */
    public void setDefault(double defaultValue){

        if (!hasDefault){
            this.defaultValue = defaultValue;
            if (RobotConstants.isTuningMode){
                SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
            }
        }
    }

    /**
     * Gets the default value for the number that has been set
     * 
     * @return The default value
     */
    public double getDefault(){
        return defaultValue;
    }

    /**
     * Publishes a new value. Value may not be returned by {@link #get()} until next  cycle.
     */

    public void set(double value){
        if (RobotConstants.isTuningMode){
            SmartDashboard.putNumber(key, value);
        } else{
            defaultValue = value;
        }
    }

    /**
     * Gets the current value from dashboard if avaiable and in tuning Mode
     * 
     * @return The current value
     */
    public double get(){
        if (!hasDefault){
            return 0.0;
        } else{
            return 
                RobotConstants.isTuningMode 
                    ? SmartDashboard.getNumber(key, defaultValue)
                    : defaultValue;
        }
    }
}

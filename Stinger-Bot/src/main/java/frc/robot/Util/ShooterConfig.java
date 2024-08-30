package frc.robot.Util;

import java.util.ArrayList;
import java.util.List;

public class ShooterConfig {
    private List<ShooterPreset> shooterConfigs;

    //Constructor
    public ShooterConfig(){
        shooterConfigs = new ArrayList<ShooterPreset>();
    }
    
    //Constructor with parameter
    public ShooterConfig(ArrayList<ShooterPreset> newShooterConfigs){
        this.shooterConfigs = newShooterConfigs;
    }

    public List<ShooterPreset> getShooterConfigs(){
        return shooterConfigs;
    }
}

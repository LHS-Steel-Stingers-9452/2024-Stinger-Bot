package frc.robot.Util;

import java.util.Collections;
import java.util.List;
public class LookUpTbl {
    ShooterConfig shooterConfig;

    private static LookUpTbl instance = new LookUpTbl();

    public LookUpTbl getInstance() {
        return instance;
    }
    
    public LookUpTbl(){
        //this creates new LookupTbl()
        //When creating a new look up table 
        shooterConfig = new ShooterConfig();
        //TODO - Put presets here
        shooterConfig.getShooterConfigs().add(new ShooterPreset(2, 0, 50));

        Collections.sort(shooterConfig.getShooterConfigs());
    }

    public ShooterPreset getShooterPreset(double DistanceFromTarget){
        int endIndex = shooterConfig.getShooterConfigs().size()-1;

        /*
         * Check if distance falls below the shortest distance in the lookup table. If the measured distance is shorter
         * select the lookup table entry with the shortest distance
         */
        if(DistanceFromTarget <= shooterConfig.getShooterConfigs().get(0).getDistance()){
            return shooterConfig.getShooterConfigs().get(0);
        }
        
        /*
         * Check if distance falls above the largest distance in the lookup table. If the measured distance is larger
         * select the lookup table entry with the largest distance
         */
        if(DistanceFromTarget >= shooterConfig.getShooterConfigs().get(endIndex).getDistance()){
            return shooterConfig.getShooterConfigs().get(endIndex);
        }

         /*
         * If the measured distance falls somewhere within the lookup table perform a binary search within the lookup
         * table
         */
        return binarySearchDistance(shooterConfig.getShooterConfigs(), 0, endIndex, DistanceFromTarget);

    }

    private ShooterPreset binarySearchDistance(List<ShooterPreset> pshooterConfig, int startIndex, int endIndex, double pDistance){
        int mid = startIndex +(endIndex - startIndex) / 2;
        double midIndexDist = pshooterConfig.get(mid).getDistance();

        //If element is present at the middle
        //return itself
        if(pDistance == midIndexDist){
            return pshooterConfig.get(mid);
        }

        //If only two elements are left
        //return the inerpolated config
        if(endIndex - startIndex == 1){
            double percentIn = 
            (pDistance - shooterConfig.getShooterConfigs().get(startIndex).getDistance()) /
                (
                    shooterConfig.getShooterConfigs().get(endIndex).getDistance() - 
                        shooterConfig.getShooterConfigs().get(startIndex).getDistance()
                );
            return interpolateShooterPreset(shooterConfig.getShooterConfigs().get(startIndex), shooterConfig.getShooterConfigs().get(endIndex), percentIn);
        }

        //If element is smaller than mid, then
        //it can only be present in left subarray
        if(pDistance < midIndexDist){
            return binarySearchDistance(pshooterConfig, startIndex, mid, pDistance);
        } else{
            //Else the element can only be present
            //in right subarray
            return binarySearchDistance(pshooterConfig, mid, endIndex, pDistance);
        }

    }

    /**
     * Obtain a new shooter preset by interpolating between two existing shooter presets
     * 
     * @param startPreset
     * @param endPreset
     * @param precetIn
     * 
     * @return new interpolated shooter preset
     */
    private ShooterPreset interpolateShooterPreset(ShooterPreset startPreset, ShooterPreset endPreset, double precetIn){
        double armAngle = startPreset.getArmAngle() + (endPreset.getArmAngle() - startPreset.getArmAngle()) *precetIn;
        double shooterVelocity = startPreset.getShooterVeloc() + (endPreset.getShooterVeloc() - startPreset.getShooterVeloc()) *precetIn;
        double distance = startPreset.getDistance() + (endPreset.getDistance() - startPreset.getDistance()) * precetIn;

        return new ShooterPreset(distance, armAngle, shooterVelocity);
    }

    /*
     * MAKE SURE YOU SORT THE LIST BEFORE CALLING THIS FUNCTION
     * @param pShooterConfig a sorted shooter config
     */
    public void setShooterConfig(ShooterConfig pShooterConfig) {
        this.shooterConfig = pShooterConfig;
    }
}

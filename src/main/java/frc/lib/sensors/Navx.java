/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.sensors;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI.Port;


public class Navx {

    private static Navx mInstance;

    public static Navx getInstance() {
        if(mInstance == null) {
            mInstance = new Navx();
        }
        return mInstance;
    }

    private AHRS navx;

    private Navx() {
        navx = new AHRS(Port.kMXP);
    }

    public double getHeading() {
        return navx.getAngle();
    }

    public void reset() {
        navx.reset();
    }


}

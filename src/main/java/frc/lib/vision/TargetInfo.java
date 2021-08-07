/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.vision;

/**
 * Add your docs here.
 */
public class TargetInfo {

    protected double x_ = 1.0;
    protected double y_;
    protected double z_;
    protected double skew_;

    public TargetInfo(double y, double z) {
        y_ = y;
        z_ = z;
    }

    public void setSkew(double skew) {
        skew_ = skew;
    }

    public double getX() {
        return x_;
    }

    public double getY() {
        return y_;
    }

    public double getZ() {
        return z_;
    }

    public double getSkew() {
        return skew_;
    }
}

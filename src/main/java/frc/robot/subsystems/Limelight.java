/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;

/**
 * Add your docs here.
 */
public class Limelight extends Subsystem {

    private static Limelight mInstance = null;
    public static Limelight getInstance() {
        if(mInstance == null) {
            mInstance = new Limelight();
        }
        return mInstance;
    }
    
    private NetworkTable mNetworkTable;

    private Limelight() {
        mNetworkTable = NetworkTableInstance.getDefault().getTable("limelight");
        mNetworkTable.getEntry("pipeline").setNumber(0);
        mNetworkTable.getEntry("stream").setNumber(0);
        mNetworkTable.getEntry("snapshot").setNumber(0);
        setLed(LedMode.OFF);
    }

    public static class PeriodicIO {
        //Inputs
        public double xOffset;
        public double yOffset;
        public double area;

        //Outputs
        public int ledMode = 1;
        public int camMode = 0;
    }

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private boolean mOutputsHaveChanged = true;

    private boolean mSeesTarget = false;
 
    @Override
    public synchronized void readPeriodicInputs() {
        if(mPeriodicIO.camMode == 0) {
            mPeriodicIO.xOffset = mNetworkTable.getEntry("tx").getDouble(0.0);
            mPeriodicIO.yOffset = mNetworkTable.getEntry("ty").getDouble(0.0);
            mSeesTarget = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;
        } else {
            mPeriodicIO.xOffset = 0.0;
            mPeriodicIO.yOffset = 0.0;
            mSeesTarget = false;
        }
        
    }

    @Override
    public synchronized void writePeriodicOutputs() {   
        if (mOutputsHaveChanged) {
            mNetworkTable.getEntry("ledMode").setNumber(mPeriodicIO.ledMode);
            mNetworkTable.getEntry("camMode").setNumber(mPeriodicIO.camMode);
            mOutputsHaveChanged = false;
        }
    }


    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putBoolean("Has Target: ", mSeesTarget);
        SmartDashboard.putBoolean("Is Close Shoot?", isCloseDistance());
    }

    public enum LedMode {
        PIPELINE, OFF, BLINK, ON;
    }

    public synchronized double getYOffset() {
        return mPeriodicIO.yOffset;
    }

    public synchronized double getXOffset() {
        return mPeriodicIO.xOffset;
    }


    public synchronized void setLed(LedMode mode) {
        if(mode.ordinal() != mPeriodicIO.ledMode) {
            mPeriodicIO.ledMode = mode.ordinal();
            mOutputsHaveChanged = true;
        }
    }

    public synchronized void setDriveMode() {
        if(mPeriodicIO.camMode != 1) {
            mPeriodicIO.camMode = 1;
            mOutputsHaveChanged = true;
        }
        setLed(LedMode.OFF);
        
    }

    public synchronized void setVisionMode() {
        if(mPeriodicIO.camMode != 0) {
            mPeriodicIO.camMode = 0;
            mOutputsHaveChanged = true;
        }
        setLed(LedMode.ON);
    }


    public synchronized void triggerOutputs() {
        mOutputsHaveChanged = true;
    }

    public synchronized boolean seesTarget() {
        return mSeesTarget;
    }

    public synchronized double getDistance() {
        double x = (Constants.kTargetHeight - Constants.kLensHeight) / 
            Math.tan(Math.toRadians(Constants.kLensHorizontalAngle + mPeriodicIO.yOffset));
        x /= Math.cos(Math.toRadians(Math.abs(mPeriodicIO.xOffset)));
        return x;
    }

    public synchronized boolean isCloseDistance() {
        return getDistance() < Constants.kFarShootDistanceThreshold;
    }

    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        return false;
    }

}

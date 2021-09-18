/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.controllers.OverridesController;
import frc.lib.util.TelemetryUtil;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;

/**
 * If the PDP drops below a certain limit, or if the entire superstructure is in motion
 * it will turn off the compressor to save potential brown outs
 */
public class AirCompressor extends Subsystem {

    private static AirCompressor mInstance = null;
    public static AirCompressor getInstance() {
        if(mInstance == null) {
            mInstance = new AirCompressor();
        }
        return mInstance;
    }

    private AirCompressor() {}

    private final Superstructure mSuperstructure = Superstructure.getInstance();
    //private final PowerDistributionPanel mPdp = new PowerDistributionPanel(Ports.PDP_ID);
    private final OverridesController mOverrides = OverridesController.getInstance();

    private boolean mIsManualControl = false;
    private final Compressor mCompressor = new Compressor(Ports.PCM_ID);

    @Override
    public void registerEnabledLoops(final ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {

            @Override
            public void onStart(final double timestamp) {
                //mPdp.clearStickyFaults();

            }

            @Override
            public void onLoop(final double timestamp) {
                synchronized (AirCompressor.this) {
                   // final boolean superstructureIsMoving = !mSuperstructure.isAtDesiredState();
                   // final boolean isBrowningOut = mPdp.getTotalCurrent() > Constants.kCompressorShutOffCurrent;

                    /*if (superstructureIsMoving || isBrowningOut || !mIsManualControl || mOverrides.airCompressorOverride.isEnabled()) {
                        stopCompressor();
                    } else {
                        startCompressor();
                    }*/
                    //startCompressor();
                    stopCompressor();

                }
            }

            @Override
            public void onStop(final double timestamp) {}

        });
    }

    private void startCompressor() {
        mCompressor.start();
    }

    private void stopCompressor() {
        mCompressor.stop();
    }

    public synchronized void setIsManualControl(boolean isManualControl) {
        mIsManualControl = isManualControl;

        if(mIsManualControl) {
            startCompressor();
        }
    }

    public synchronized boolean isManualControl() {
        return mIsManualControl;
    }



    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {

        System.out.println("//////// Testing Air Compressor ////////");
        mCompressor.start();
        Timer.delay(0.5);
        mCompressor.stop();
        
        return true;
    }

    @Override
    public void outputTelemetry() {

    }
}

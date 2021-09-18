/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

public class OverridesController {

    private static OverridesController mInstance = null;
    public static OverridesController getInstance() {
        if(mInstance == null) {
            mInstance = new OverridesController();
        }
        return mInstance;
    }

    private final Joystick mJoystick;

    public final Override visionOverride, shooterClosedLoopOverride, idleShooterOverride,
        spinnerColorSensorOverride, spinnerEncoderOverride, airCompressorOverride, indexSensorOverride;

    private final int VISION = 1; //arduino port: 4
    private final int SHOOTER_CLOSED_LOOP = 2; //arduino port: 5
    private final int IDLE_SHOOTER = 3; //arduino port: 3
    private final int COLOR_SENSOR = 4; //arduino port: 2
    private final int SPINNER_ENCODER = 5; //arduino port: 10
    private final int AIR_COMPRESSOR = 6; //arduino port: 11
    private final int INDEX_SENSOR = 9; //arduino port: 12


    private OverridesController() {
        mJoystick = new Joystick(2);

        visionOverride = new Override(VISION);
        shooterClosedLoopOverride = new Override(SHOOTER_CLOSED_LOOP);
        idleShooterOverride = new Override(IDLE_SHOOTER);
        spinnerColorSensorOverride = new Override(COLOR_SENSOR);
        spinnerEncoderOverride = new Override(SPINNER_ENCODER);
        airCompressorOverride = new Override(AIR_COMPRESSOR);
        indexSensorOverride = new Override(INDEX_SENSOR);
    }


    public class Override {

        private final int mButtonNumber;
        public Override(int buttonNumber) {
            mButtonNumber = buttonNumber;
        }

        private boolean mPrevIsActive = false;
        private double mStateChangedTimestamp = 0.0;
        private double mStateChangeTimeThreshold = 0.1;

        private boolean mIsEnabled = false;

        public boolean isEnabled() {
            return mIsEnabled;
        }


        public void update() {
            boolean isActive = mJoystick.getRawButton(mButtonNumber);

            if(isActive) {
                if(!mPrevIsActive) {
                    mStateChangedTimestamp = Timer.getFPGATimestamp();
                } else {
                    mIsEnabled = Timer.getFPGATimestamp() - mStateChangedTimestamp > mStateChangeTimeThreshold;
                }
            } else {
                if(mPrevIsActive) {
                    mStateChangedTimestamp = Timer.getFPGATimestamp();
                } else {
                    mIsEnabled = !(Timer.getFPGATimestamp() - mStateChangedTimestamp > mStateChangeTimeThreshold);
                }
            }

            mPrevIsActive = isActive;
        }

    }

    public void update() {
        visionOverride.update();
        shooterClosedLoopOverride.update();
        airCompressorOverride.update();
        spinnerColorSensorOverride.update();
        spinnerEncoderOverride.update();
    }

}

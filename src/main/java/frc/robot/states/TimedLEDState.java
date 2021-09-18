/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.states;

/**
 * Add your docs here.
 */
public interface TimedLEDState {
    void getCurrentLEDState(LEDState desiredState, double timestamp);
    int getPriority();

    class FlashingLEDState implements TimedLEDState {

        public static FlashingLEDState kBootFault = new FlashingLEDState(LEDState.kRed, 0.75, 1);
        public static FlashingLEDState kRuntimeFault = new FlashingLEDState(LEDState.kRed, 0.25, 1);

        public static FlashingLEDState kShooterAligning = new FlashingLEDState(LEDState.kGreen, 0.25, 3);
        public static FlashingLEDState kTargetNotVisible = new FlashingLEDState(LEDState.kBlue, 0.25, 3);

        public static FlashingLEDState kClimbNotBalanced = new FlashingLEDState(LEDState.kWhite, LEDState.kRed, 0.4, 2);
        public static FlashingLEDState kClimbIsBalanced = new FlashingLEDState(LEDState.kWhite, LEDState.kGreen, 0.4, 3);

        public static FlashingLEDState kIntakingWithVision = new FlashingLEDState(LEDState.kYellow, LEDState.kGreen, 0.25, 3);
        public static FlashingLEDState kIntakingWithoutVision = new FlashingLEDState(LEDState.kYellow, 0.25, 3);

        public static FlashingLEDState kPathFollowing = new FlashingLEDState(LEDState.kOrange, 0.25, 2);
        public static FlashingLEDState kTestMode = new FlashingLEDState(LEDState.kOrange, LEDState.kGreen, 0.25, 3);
        public static FlashingLEDState kManualOverrideEngaged = new FlashingLEDState(LEDState.kPurple, 0.25, 1);


        LEDState mStateOne = new LEDState(0.0, 0.0, 0.0);
        LEDState mStateTwo = new LEDState(0.0, 0.0, 0.0);
        double mDuration;
        int mPriority;

        public FlashingLEDState(LEDState stateOne, LEDState stateTwo, double duration, int priority) {
            mStateOne = stateOne;
            mStateTwo = stateTwo;
            mDuration = duration;
            mPriority = priority;
        }

        public FlashingLEDState(LEDState state, double duration, int priority) {
            this(LEDState.kOff, state, duration, priority);
        }

        /**
         * This turns the first parameter (LEDState desiredState) into a copy of the current state (based off of a modulus operator that checks which color is currently active) of the FlashingLEDState LEDState.
         * Then use this first parameter as the object to read current RGB values from.
         * Do what you want with these values wherever you call this method.
         * Note that if you want the lights to flash, this method will have to be called within another method that is constantly being updated.
         *
         * @param desiredState The LEDState that will be MODIFIED to become the same as the current state of the FlashingLEDState LEDState
         */
        @Override
        public void getCurrentLEDState(LEDState desiredState, double timestamp) {
            if((int) (timestamp / mDuration) % 2 == 0) {
                desiredState.copyFrom(mStateOne);
            } else {
                desiredState.copyFrom(mStateTwo);
            }

        }

        @Override
        public int getPriority() {
            return mPriority - 1;
        }

    }

    class StaticLEDState implements TimedLEDState {

        public static StaticLEDState kEmergency = new StaticLEDState(LEDState.kRed, 1);
        public static StaticLEDState kShooting = new StaticLEDState(LEDState.kGreen, 3);
        public static StaticLEDState kAutoComplete = new StaticLEDState(LEDState.kOrange, 2);
        public static StaticLEDState kHopperFull = new StaticLEDState(LEDState.kYellow, 3);
        public static StaticLEDState kDisabled = new StaticLEDState(LEDState.kOff, 1);
        public static StaticLEDState kEnabled = new StaticLEDState(LEDState.kWhite, 3);
        public static StaticLEDState kBasic = new StaticLEDState(LEDState.kSpartaGreen, 3);

        LEDState mStaticState = new LEDState(0.0, 0.0, 0.0);
        int mPriority;

        public StaticLEDState(LEDState staticState, int priority) {
            mStaticState.copyFrom(staticState);
            mPriority = priority;
        }

        /**
         * This turns the first parameter (LEDState desiredState) into a copy of the current state of the StaticLEDState LEDState.
         * Then use this first parameter as the object to read current RGB values from.
         * Do what you want with these values wherever you call this method.
         *
         * @param desiredState The LEDState that will be MODIFIED to become the same as the current state of the SteadyLEDState LEDState
         */
        @Override
        public void getCurrentLEDState(LEDState desiredState, double timestamp) {
            desiredState.copyFrom(mStaticState);
        }

        @Override
        public int getPriority() {
            return mPriority - 1;
        }

    }


}

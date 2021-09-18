/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.controllers;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.util.*;

/**
 * Add your docs here.
 */
public class Xbox extends XboxController {
    public Button aButton, bButton, xButton, yButton, startButton, backButton,
        leftBumper, rightBumper, leftJoystickButton, rightJoystickButton, leftTrigger,
        rightTrigger, dpadUp, dpadRight, dpadLeft, dpadDown;

    private final int A_BUTTON = 1;
    private final int B_BUTTON = 2;
    private final int X_BUTTON = 3;
    private final int Y_BUTTON = 4;
    private final int LEFT_BUMPER = 5;
    private final int RIGHT_BUMPER = 6;
    private final int BACK_BUTTON = 7;
    private final int START_BUTTON = 8;
    private final int RIGHT_JOYSTICK_BUTTON = 10;
    private final int LEFT_JOYSTICK_BUTTON = 9;
    private final int DPAD_UP = -1;
    private final int DPAD_RIGHT = -2;
    private final int DPAD_DOWN = -3;
    private final int DPAD_LEFT = -4;
    private final int LEFT_TRIGGER = -5;
    private final int RIGHT_TRIGGER = -6; 

    private final int LEFT_X_AXIS = 0;
    private final int RIGHT_X_AXIS = 4;
    private final int LEFT_Y_AXIS = 1;
    private final int RIGHT_Y_AXIS = 5;
    private final int LEFT_TRIGGER_AXIS = 2;
    private final int RIGHT_TRIGGER_AXIS = 3;

    private double TRIGGER_PRESS_THRESHOLD = 0.05;

    public void setTriggerPressThreshold(double pressThreshold) {
        TRIGGER_PRESS_THRESHOLD = pressThreshold;
    }

    private double DEAD_BAND = 0.05;

    public void setDeadband(double deadband) {
        DEAD_BAND = deadband;
    }

    private boolean rumbling = false;

    public Xbox(int port) {
        super(port);
    
        aButton = new Button(A_BUTTON);        
        bButton = new Button(B_BUTTON);
        xButton = new Button(X_BUTTON);
        yButton = new Button(Y_BUTTON);
        startButton = new Button(START_BUTTON);
        backButton = new Button(BACK_BUTTON);
        leftBumper = new Button(LEFT_BUMPER);
        rightBumper = new Button(RIGHT_BUMPER);
        leftJoystickButton = new Button(LEFT_JOYSTICK_BUTTON);
        rightJoystickButton = new Button(RIGHT_JOYSTICK_BUTTON);
        leftTrigger = new Button(LEFT_TRIGGER);
        rightTrigger = new Button(RIGHT_TRIGGER);
        dpadUp = new Button(DPAD_UP);
        dpadRight = new Button(DPAD_RIGHT);
        dpadDown = new Button(DPAD_DOWN);
        dpadLeft = new Button(DPAD_LEFT);
    }

    @Override
    public double getX(Hand hand) {
        if(hand.equals(Hand.kLeft)) {
            return Util.deadBand(getRawAxis(LEFT_X_AXIS), DEAD_BAND);
        } else {
            return Util.deadBand(getRawAxis(RIGHT_X_AXIS), DEAD_BAND);
        }
    }

    @Override
    public double getY(Hand hand) {
        if(hand.equals(Hand.kLeft)) {
            return -Util.deadBand(getRawAxis(LEFT_Y_AXIS), DEAD_BAND);
        } else {
            return -Util.deadBand(getRawAxis(RIGHT_Y_AXIS), DEAD_BAND);
        }
    }

    @Override
    public double getTriggerAxis(Hand hand) {
        if(hand.equals(Hand.kLeft)) {
            return Util.deadBand(getRawAxis(LEFT_TRIGGER_AXIS), DEAD_BAND);
        } else {
            return Util.deadBand(getRawAxis(RIGHT_TRIGGER_AXIS), DEAD_BAND);
        }
    }
        


    public class Button {
        private boolean buttonCheck = false;
        private boolean buttonActive = false;
        private boolean activationReported = false;
        private boolean longPressed = false;
        private boolean longPressActivated = false;
        private boolean hasBeenPressed = false;
        private boolean longReleased = false;
    
        private double buttonPressStartTime = 0;

        private double longPressDuration = 0.25;

        public void setLongPressDuration(double longPressDuration) {
            this.longPressDuration = longPressDuration;
        }

        private int buttonNumber;
        
        public Button(int buttonNumber) {
            this.buttonNumber = buttonNumber;
        }

        public void update() {
            if(buttonNumber > 0) {
                buttonCheck = getRawButton(buttonNumber);
            } else {
                switch(buttonNumber) { //TODO: Add logic for pov other mid points
                    case LEFT_TRIGGER:
                        buttonCheck = getTriggerAxis(Hand.kLeft) > TRIGGER_PRESS_THRESHOLD;
                        break;
                    case RIGHT_TRIGGER:
                        buttonCheck = getTriggerAxis(Hand.kRight) > TRIGGER_PRESS_THRESHOLD;
                        break;
                    case DPAD_UP:
                        buttonCheck = (getPOV() == 0);
                        break;
                    case DPAD_RIGHT:
                        buttonCheck = (getPOV() == 90);
                        break;
                    case DPAD_DOWN:
                        buttonCheck = (getPOV() == 180);
                        break;
                    case DPAD_LEFT:
                        buttonCheck = (getPOV() == 270);
                        break;
                    default:
                        buttonCheck = false;
                        break;
                }
            }

            if(buttonCheck) {
                if(buttonActive) {
                    if((Timer.getFPGATimestamp() - buttonPressStartTime) > longPressDuration && !longPressActivated) {
                        longPressActivated = true;
                        longPressed = true;
                        longReleased = false;
                    }
                } else {
                    buttonActive = true;
                    activationReported = false;
                    buttonPressStartTime = Timer.getFPGATimestamp();
                }
            } else {
                if(buttonActive) {
                    buttonActive = false;
                    activationReported = true;
                    if(longPressActivated) {
                        hasBeenPressed = false;
                        longPressActivated = false;
                        longPressed = false;
                        longReleased = true;
                    } else {
                        hasBeenPressed = true;
                    }
                }
            }
        }

        public boolean wasActivated() {
            if(buttonActive && !activationReported) {
                activationReported = true;
                return true;
            }
            return false;
        }

        
        public boolean shortReleased() {
            if(hasBeenPressed) {
                hasBeenPressed = false;
                return true;
            }
            return false;
        }

        
        public boolean longPressed() {
            if(longPressed) {
                longPressed = false;
                return true;
            }
            return false;
        }


        public boolean longReleased() {
            if(longReleased) {
                longReleased = false;
                return true;
            }
            return false;
        }

        public boolean wasReleased() {
            return shortReleased() || longReleased();
        }

        public boolean isBeingPressed() {
            return buttonActive;
        }
    }

    public void update() {
        aButton.update();
        bButton.update();
        xButton.update();
        yButton.update();
        startButton.update();
        backButton.update();
        leftBumper.update();
        rightBumper.update();
        leftJoystickButton.update();
        rightJoystickButton.update();
        leftTrigger.update();
        rightTrigger.update();
        dpadUp.update();
        dpadRight.update();
        dpadDown.update();
        dpadLeft.update();
    }

    /*public void rumble(double rumblesPerSecond, double seconds) {
        if(!rumbling) {
            RumbleThread r = new RumbleThread(rumblesPerSecond, seconds);
            r.start();
        }
    }*/

    public void rumble(double rumblePower) {
        setRumble(RumbleType.kLeftRumble, rumblePower);
        setRumble(RumbleType.kRightRumble, rumblePower);
    }

    public boolean isRumbling() {
        return rumbling;
    }


    /*public class RumbleThread extends Thread {
        private double startTime = 0;
        private double seconds;
        private long interval;

        public RumbleThread(double rumblesPerSecond, double seconds) {
            this.seconds = seconds;
            interval = (long) (1/(rumblesPerSecond*2)*1000);
        }

        public void run() {
            rumbling = true;
            startTime = Timer.getFPGATimestamp();
            try {
                while((Timer.getFPGATimestamp() - startTime) < seconds) {
                    setRumble(RumbleType.kLeftRumble, 1);
                    setRumble(RumbleType.kRightRumble, 1);
                    sleep(interval);
                    setRumble(RumbleType.kLeftRumble, 0);
                    setRumble(RumbleType.kRightRumble, 0);
                    sleep(interval);
                }
            } catch (InterruptedException e) {
                rumbling = false;
                e.printStackTrace();
            }
            rumbling = false;
        }
    }*/


    
}

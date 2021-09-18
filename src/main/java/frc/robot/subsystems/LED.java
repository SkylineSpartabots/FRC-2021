/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.TelemetryUtil;
import frc.lib.util.TelemetryUtil.PrintStyle;
import frc.robot.Ports;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.states.LEDState;
import frc.robot.states.TimedLEDState;

/**
 * Add your docs here.
 */
public class LED extends Subsystem {

    private static LED mInstance = null;

    public static LED getInstance() {
        if (mInstance == null) {
            mInstance = new LED();
        }
        return mInstance;
    }

    // hardware
    private final CANifier mCanifier;

    // states
    private ArrayList<TimedLEDState> errorstates = new ArrayList<>();
    private ArrayList<TimedLEDState> warningstates = new ArrayList<>();
    private ArrayList<TimedLEDState> infostates = new ArrayList<>();
    private List<ArrayList<TimedLEDState>> allStates = Arrays.asList(errorstates, warningstates, infostates);
    private LEDState mDesiredLEDState = new LEDState(0.0, 0.0, 0.0);
    private int priorityLogic = 0;
    private int cycleLogic = 0;
    private double cycletime = 1000;

    private LED() {
        mCanifier = new CANifier(Ports.CANIFIER_ID);
    }

    public void addForemostActiveState(TimedLEDState timedState) {
        allStates.get(timedState.getPriority()).add(0, timedState);
    }

    public void addStateToQueue(TimedLEDState timedState) {
        allStates.get(timedState.getPriority()).add(timedState);
    }

    public void removeForemostState(int priorityLevel) {
        allStates.get(priorityLevel - 1).remove(0);
    }

    public void removeFromQueue(TimedLEDState timedState) {
        allStates.get(timedState.getPriority()).remove(timedState);
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            double startTime = Timer.getFPGATimestamp();

            @Override
            public void onStart(double timestamp) {
                mCanifier.setLEDOutput(0, LEDChannel.LEDChannelA);
                mCanifier.setLEDOutput(0, LEDChannel.LEDChannelB);
                mCanifier.setLEDOutput(0, LEDChannel.LEDChannelC);
                addStateToQueue(TimedLEDState.StaticLEDState.kBasic);
                logics(startTime);
            }

            @Override
            public void onLoop(double timestamp) {
                logics(startTime);
                updateLED(timestamp);
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    public void logics(double startTime) {
        if (errorstates.size() == 0) {
            if (warningstates.size() == 0) {
                priorityLogic = 2;
            } else {
                priorityLogic = 1;
            }
        } else {
            priorityLogic = 0;
        }
        if (Timer.getFPGATimestamp() - startTime > cycletime) {
            startTime = Timer.getFPGATimestamp();
            cycleLogic++;
            if (allStates.get(priorityLogic).size() < cycleLogic) {
                cycleLogic = 0;
            }
        }
    }

    public void updateLED(double timestamp) {
        allStates.get(priorityLogic).get(cycleLogic).getCurrentLEDState(mDesiredLEDState, timestamp);
    }

    @Override
    public void writePeriodicOutputs() {
        mCanifier.setLEDOutput(mDesiredLEDState.red, LEDChannel.LEDChannelA);
        mCanifier.setLEDOutput(mDesiredLEDState.green, LEDChannel.LEDChannelB);
        mCanifier.setLEDOutput(mDesiredLEDState.blue, LEDChannel.LEDChannelC);
    }

    @Override
    public void stop() {
        mCanifier.setLEDOutput(0, LEDChannel.LEDChannelA);
        mCanifier.setLEDOutput(0, LEDChannel.LEDChannelB);
        mCanifier.setLEDOutput(0, LEDChannel.LEDChannelC);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        TelemetryUtil.print("Current RGB Values: " + mDesiredLEDState.red + ", " + mDesiredLEDState.green + ", "
                + mDesiredLEDState.blue, PrintStyle.INFO, true);
    }
}

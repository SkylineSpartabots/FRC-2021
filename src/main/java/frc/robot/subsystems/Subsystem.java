/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.loops.ILooper;

/**
 * Add your docs here.
 */
public abstract class Subsystem {

    public void writeToLog() {}

    public void readPeriodicInputs() {}

    public void writePeriodicOutputs() {}

    public void registerEnabledLoops(ILooper mEnabledLooper) {}

    public void zeroSensors() {}

    public abstract void stop();

    public abstract boolean checkSystem();

    // where subsystems update telemetry, gets updated in outputTelemetry
    public abstract void updateTelemetry();

    public final void outputTelemetry() {
        updateTelemetry();
        try {
            Dashboard.parse(outputTelemetry, buttons);
        } catch (Exception e) {
            throw new RuntimeException("Error while parsing telemetry", e);
        }
    }

    // add buttons here
    public abstract void initTelemetry();

    public HashMap<String, Object> outputTelemetry = new HashMap<>();

    public HashMap<String, Button> buttons = new HashMap<>();

    public boolean hasEmergency = false;
}

// literally just testing to see if my git works well. also, HashMap<Integer, Object> vars = new HashMap<Integer, Object>();
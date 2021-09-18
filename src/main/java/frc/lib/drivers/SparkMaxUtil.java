/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.drivers;

import com.revrobotics.CANError;

import frc.lib.util.TelemetryUtil;
import frc.lib.util.TelemetryUtil.PrintStyle;

/**
 * Add your docs here.
 */
public class SparkMaxUtil {

    public static void checkError(CANError errorCode, String message, boolean log) {
        if (errorCode != CANError.kOk) {
            TelemetryUtil.print(message + " " + errorCode, PrintStyle.ERROR, log);
        }
    }

    public static void setCurrentLimit(LazySparkMax sparkMax, int amps, int maxAllowableCurrent) {
        checkError(sparkMax.setSmartCurrentLimit(amps), 
                sparkMax.getName() + " failed to set current limit", true);
        checkError(sparkMax.setSecondaryCurrentLimit(maxAllowableCurrent), 
                sparkMax.getName() + " failed to set max allowable current", true);
    }

    public static void disableCurrentLimit(LazySparkMax sparkMax) {
        setCurrentLimit(sparkMax, 0, 0);
    }

    public static void setVoltageCompensation(LazySparkMax sparkMax, double voltage) {
        checkError(sparkMax.enableVoltageCompensation(voltage), 
                sparkMax.getName() + " failed to enable voltage comp.", true);
    }

    public static void disableVoltageCompensation(LazySparkMax sparkMax) {
        checkError(sparkMax.disableVoltageCompensation(), 
            sparkMax.getName() + " failed to disable voltage comp.", true);
    }
}

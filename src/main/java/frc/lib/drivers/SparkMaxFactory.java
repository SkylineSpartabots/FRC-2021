/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.drivers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class SparkMaxFactory {

    public static class Configuration {
        public int TIME_OUT_MS = Constants.kTimeOutMs;
        public IdleMode NEUTRAL_MODE = IdleMode.kCoast;
        public boolean INVERTED = false;

        public int STATUS_FRAME_RATE_0 = 10;
        public int STATUS_FRAME_RATE_1 = 1000;
        public int STATUS_FRAME_RATE_2 = 1000;

        public double OPEN_LOOP_RAMP_RATE = 0.0;
        public double CLOSED_LOOP_RAMP_RATE = 0.0;

        public boolean ENABLE_VOLTAGE_COMPENSATION = true;
        public double NOMINAL_VOLTAGE = 12.0;

        public int CURRENT_LIMIT = 35;
        public int MAX_ALLOWABLE_CURRENT = 50;
        public boolean ENABLE_CURRENT_LIMIT = false;
    }

    private static final Configuration kDefaultConfiguration = new Configuration();
    private static final Configuration kSlaveConfiguration = new Configuration();

    static {
        kSlaveConfiguration.STATUS_FRAME_RATE_0 = 1000;
        kSlaveConfiguration.STATUS_FRAME_RATE_1 = 1000;
        kSlaveConfiguration.STATUS_FRAME_RATE_2 = 1000;
    }

    public static LazySparkMax createDefaultSparkMax(String name, int deviceID, boolean isInverted) {
        final LazySparkMax sparkMax = createSparkMax(name, deviceID, kDefaultConfiguration);
        sparkMax.setInverted(isInverted);
        return sparkMax;
    }

    public static LazySparkMax createSlaveSparkMax(String name, int deviceID, CANSparkMax master, boolean isInverted) {
        final LazySparkMax sparkMax = createSparkMax(name, deviceID, kSlaveConfiguration);
        SparkMaxUtil.checkError(sparkMax.follow(master, isInverted), sparkMax.getName() + " failed to set" +
            "master following on init", true);
        return sparkMax;
    }

    public static LazySparkMax createSparkMax(String name, int deviceID, Configuration config) {
        Timer.delay(0.25);
        LazySparkMax sparkMax = new LazySparkMax(name, deviceID);
        
        SparkMaxUtil.checkError(sparkMax.setCANTimeout(config.TIME_OUT_MS), 
            sparkMax.getName() + " failed to set CAN timeout on init", true);
        
        sparkMax.set(ControlType.kDutyCycle, 0.0);

        SparkMaxUtil.checkError(sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0
            , config.STATUS_FRAME_RATE_0), sparkMax.getName() + " failed to set status 0 rate on init", true);
        
        SparkMaxUtil.checkError(sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1
            , config.STATUS_FRAME_RATE_1), sparkMax.getName() + " failed to set status 1 rate on init", true);
        
        SparkMaxUtil.checkError(sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2
            , config.STATUS_FRAME_RATE_0), sparkMax.getName() + " failed to set status 0 rate on init", true);
        
        sparkMax.clearFaults();

        SparkMaxUtil.checkError(sparkMax.setIdleMode(config.NEUTRAL_MODE), sparkMax.getName() + 
            " failed to set neutral mode on init", true);
        
        SparkMaxUtil.checkError(sparkMax.setOpenLoopRampRate(config.OPEN_LOOP_RAMP_RATE), 
            sparkMax.getName() + " failed to set open loop ramp on init", true);
        
        SparkMaxUtil.checkError(sparkMax.setClosedLoopRampRate(config.CLOSED_LOOP_RAMP_RATE), 
            sparkMax.getName() + " failed to set closed loop ramp on init", true);
        
        if(config.ENABLE_VOLTAGE_COMPENSATION) {
            SparkMaxUtil.checkError(sparkMax.enableVoltageCompensation(config.NOMINAL_VOLTAGE), 
                sparkMax.getName() + " failed to enable voltage comp. on init", true);
        } else {
            SparkMaxUtil.checkError(sparkMax.disableVoltageCompensation(), sparkMax.getName() + 
                " failed to disable voltage comp. on init", true);
        }

        if(config.ENABLE_CURRENT_LIMIT) {
            SparkMaxUtil.checkError(sparkMax.setSmartCurrentLimit(config.CURRENT_LIMIT), 
                sparkMax.getName() + " failed to set current limit on init", true);
            SparkMaxUtil.checkError(sparkMax.setSecondaryCurrentLimit(config.MAX_ALLOWABLE_CURRENT), 
                sparkMax.getName() + " failed to set max allowable current on init", true);
        } else {
            SparkMaxUtil.checkError(sparkMax.setSmartCurrentLimit(0), sparkMax.getName() 
                + " failed to disable current limit on init", true);
            SparkMaxUtil.checkError(sparkMax.setSecondaryCurrentLimit(0), 
                sparkMax.getName() + " failed to disable max allowable current on init", true);

        }

        return sparkMax;
    }
}

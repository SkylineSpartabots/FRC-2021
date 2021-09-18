/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.drivers;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class CANCoderFactory {

    private static int kTimeOutMs = Constants.kTimeOutMs;

    public static class Configuration {

        public SensorVelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD = SensorVelocityMeasPeriod.Period_100Ms;
        public int VELOCITY_MEASUREMENT_WINDOW = 64;

        public SensorInitializationStrategy INITIALIZATION_STRATEGY = SensorInitializationStrategy.BootToZero;

        public double SENSOR_COEFFECIENT = 0.087890625;
        public String UNIT_STRING = "deg";
        public SensorTimeBase SENSOR_TIME_BASE = SensorTimeBase.PerSecond;

        public int SENSOR_DATA_FRAME_RATE_MS = 5;
        public int FAULTS_FRAME_RATE_MS = 100;
    }

    private static final Configuration kDefaultConfiguration = new Configuration();
    private static final Configuration kDriveConfiguration = new Configuration();

    static {
        kDriveConfiguration.UNIT_STRING = "inches";
        //TODO: Logic for determinig constant given wheel diameter to give inches as native unit
        kDefaultConfiguration.SENSOR_COEFFECIENT = (Constants.kDriveWheelDiameter * Math.PI) / 360.0; 
        kDriveConfiguration.SENSOR_TIME_BASE = SensorTimeBase.Per100Ms_Legacy;
    }

    public static CANCoder createDefaultEncoder(String name, int id, boolean sensorPhase) {
        return createEncoder(name, id, kDefaultConfiguration, sensorPhase);
    }

    public static CANCoder createDriveEncoder(String name, int id, boolean sensorPhase) {
        return createEncoder(name, id, kDriveConfiguration, sensorPhase);
    }
    
    public static CANCoder createEncoder(String name, int id, Configuration config, boolean sensorPhase) {
        CANCoder encoder = new CANCoder(id);

        PheonixUtil.checkError(encoder.setPosition(0, kTimeOutMs),
             "failed to reset encoder for " + name, true);

        PheonixUtil.checkError(encoder.configSensorDirection(sensorPhase, kTimeOutMs),
             "failed to set sensor phase for " + name, true);

        PheonixUtil.checkError(encoder.configVelocityMeasurementPeriod(config.VELOCITY_MEASUREMENT_PERIOD, kTimeOutMs),
             "Failed to set velocity measurement period for " + name, true);
        
        PheonixUtil.checkError(encoder.configVelocityMeasurementWindow(config.VELOCITY_MEASUREMENT_WINDOW, kTimeOutMs),
             "Failed to set velocity measurement window for " + name, true);

        PheonixUtil.checkError(encoder.configFeedbackCoefficient(config.SENSOR_COEFFECIENT, 
            config.UNIT_STRING, config.SENSOR_TIME_BASE, kTimeOutMs), "Failed to set coeffecient and other for " + name, true);
        
        PheonixUtil.checkError(encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData,
             config.SENSOR_DATA_FRAME_RATE_MS, kTimeOutMs), "Failed to set sensor data frame rate for " + name, true);

        PheonixUtil.checkError(encoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 
            config.FAULTS_FRAME_RATE_MS, kTimeOutMs), "Failed to set faults frame rate for " + name, true);

        return encoder;
    }
}

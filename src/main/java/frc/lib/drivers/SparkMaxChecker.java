/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.drivers;

import java.util.ArrayList;

import com.revrobotics.ControlType;

import frc.robot.subsystems.Subsystem;


public class SparkMaxChecker extends MotorChecker<LazySparkMax> {

    private static class StoredSparkConfiguration {
        public LazySparkMax mMaster = null;
        public ControlType mMode;
        public double mSetValue;
    }

    protected ArrayList<StoredSparkConfiguration> mStoredConfigurations = new ArrayList<>();
    

    public static boolean checkMotors(Subsystem subsystem,
                                      ArrayList<MotorConfig<LazySparkMax>> motorsToCheck,
                                      CheckerConfig checkerConfig) {
        SparkMaxChecker checker = new SparkMaxChecker();
        return checker.checkMotorImpl(subsystem, motorsToCheck, checkerConfig); 
    }

    public static void testMotors(Subsystem subsystem, 
                                    ArrayList<MotorConfig<LazySparkMax>> motorsToCheck,
                                    TesterConfig testerConfig) {
        SparkMaxChecker checker = new SparkMaxChecker();
        checker.testMotorConfigurationImpl(subsystem, motorsToCheck, testerConfig);
    }
    
    @Override
    protected void storeConfiguration() {
        for (MotorConfig<LazySparkMax> config : mMotorsToCheck) {

            StoredSparkConfiguration configuration = new StoredSparkConfiguration();
            configuration.mMaster = config.mMotor.getMaster();
            configuration.mMode = config.mMotor.getControlMode();
            configuration.mSetValue = config.mMotor.getLastSet();

            mStoredConfigurations.add(configuration);
            config.mMotor.restoreFactoryDefaults();
        }
    }

    @Override
    protected void restoreConfiguration() {
        for (int i = 0; i < mMotorsToCheck.size(); i++) {
            mMotorsToCheck.get(i).mMotor.set(mStoredConfigurations.get(i).mMode,
                    mStoredConfigurations.get(i).mSetValue);

            if (mStoredConfigurations.get(i).mMaster != null) {
                mMotorsToCheck.get(i).mMotor.follow(mStoredConfigurations.get(i).mMaster);
            }
        }
    }   

    @Override
    protected void setMotorOutput(LazySparkMax motor, double output) {
        motor.set(ControlType.kDutyCycle, output);
    }

    @Override
    protected double getMotorCurrent(LazySparkMax motor) {
        return motor.getOutputCurrent();
    }

    @Override
    protected String getMotorName(LazySparkMax motor) {
        return motor.getName();
    }
}

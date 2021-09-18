/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.drivers;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.subsystems.Subsystem;


public class TalonSRXChecker extends MotorChecker<LazyTalonSRX> {

    private static class StoredTalonSRXConfiguration {
        public LazyTalonSRX mMaster;
        public ControlMode mMode;
        public double mSetValue;
    }

    protected ArrayList<StoredTalonSRXConfiguration> mStoredConfigurations = new ArrayList<>();

    public static boolean checkMotors(Subsystem subsystem, ArrayList<MotorConfig<LazyTalonSRX>> motorsToCheck,
                                        CheckerConfig checkerConfig) {
        TalonSRXChecker checker = new TalonSRXChecker();
        return checker.checkMotorImpl(subsystem, motorsToCheck, checkerConfig);                                     
    }

    public static void testMotors(Subsystem subsystem, 
                                    ArrayList<MotorConfig<LazyTalonSRX>> motorsToCheck,
                                    TesterConfig testerConfig) {
        TalonSRXChecker checker = new TalonSRXChecker();
        checker.testMotorConfigurationImpl(subsystem, motorsToCheck, testerConfig);
    }

    @Override
    protected void storeConfiguration() {
        for (MotorConfig<LazyTalonSRX> config : mMotorsToCheck) {
            StoredTalonSRXConfiguration configuration = new StoredTalonSRXConfiguration();
            configuration.mMode = config.mMotor.getControlMode();
            configuration.mSetValue = config.mMotor.getLastSet();
            configuration.mMaster = config.mMotor.getMaster();

            mStoredConfigurations.add(configuration);
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
    protected void setMotorOutput(LazyTalonSRX motor, double output) {
        motor.set(ControlMode.PercentOutput, output);
    }

    @Override
    protected double getMotorCurrent(LazyTalonSRX motor) {
        return motor.getStatorCurrent();
    }

    @Override
    protected String getMotorName(LazyTalonSRX motor) {
        return motor.getName();
    }
}

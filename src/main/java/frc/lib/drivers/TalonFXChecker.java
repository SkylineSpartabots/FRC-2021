/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.drivers;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import frc.robot.subsystems.Subsystem;


public class TalonFXChecker extends MotorChecker<LazyTalonFX> {

    private static class StoredTalonFXConfiguration {
        public LazyTalonFX mMaster = null;
        public ControlMode mMode;
        public double mSetValue;
    }


    protected ArrayList<StoredTalonFXConfiguration> mStoredConfigurations = new ArrayList<>();


    public static boolean checkMotors(Subsystem subsystem,
                                        ArrayList<MotorConfig<LazyTalonFX>> motorsToCheck,
                                        CheckerConfig checkerConfig) {
        TalonFXChecker checker = new TalonFXChecker();
        return checker.checkMotorImpl(subsystem, motorsToCheck, checkerConfig);
    }

    public static void testMotors(Subsystem subsystem, 
                                    ArrayList<MotorConfig<LazyTalonFX>> motorsToCheck,
                                    TesterConfig testerConfig) {
        TalonFXChecker checker = new TalonFXChecker();
        checker.testMotorConfigurationImpl(subsystem, motorsToCheck, testerConfig);
    }

    @Override
    protected void storeConfiguration() {
        for (MotorConfig<LazyTalonFX> config : mMotorsToCheck) {
            StoredTalonFXConfiguration configuration = new StoredTalonFXConfiguration();
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
    protected void setMotorOutput(LazyTalonFX motor, double output) {
        motor.set(TalonFXControlMode.PercentOutput, output);
    }

    @Override
    protected double getMotorCurrent(LazyTalonFX motor) {
        return motor.getStatorCurrent();
    }

    @Override
    protected String getMotorName(LazyTalonFX motor) {
        return motor.getName();
    }
}

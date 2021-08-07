/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.drivers;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.Util;
import frc.robot.subsystems.Subsystem;


public abstract class MotorChecker<T> {
    public static class CheckerConfig {
        public double mCurrentFloor = 5;
        public double mRPMFloor = 2000;

        public double mCurrentEpsilon = 5.0;
        public double mRPMEpsilon = 500;
        public DoubleSupplier mRPMSupplier = null;

        public double mRuntime = 4.0;
        public double mWaittime = 2.0;
        public double mOutputPercent = 0.5;
    }

    public static class TesterConfig {
        public DoubleSupplier mRPMSupplier = null;
        public double mRuntime = 4.0;
        public double mWaittime = 2.0;
        public double mOutputPercent = 0.5;
    }

    public static class MotorConfig<T> {
        public T mMotor;

        public MotorConfig(T motor) {
            mMotor = motor;
        }
    }

    protected ArrayList<MotorConfig<T>> mMotorsToCheck;

    protected abstract void storeConfiguration();
    
    protected abstract void restoreConfiguration();

    protected abstract void setMotorOutput(T motor, double output);

    protected abstract double getMotorCurrent(T motor);

    protected abstract String getMotorName(T motor);

    protected void testMotorConfigurationImpl(Subsystem subsystem, 
                                          ArrayList<MotorConfig<T>> motorsToCheck,
                                          TesterConfig testerConfig) {
        System.out.println("/////////////////////////////////////////");
        System.out.println("Testing Subsystem: " + subsystem.getClass() + 
            "for " + motorsToCheck.size() + " motors");


        if(motorsToCheck.isEmpty()) {
            System.out.println("Empty motor array passed");
            return;
        }
        
        ArrayList<Double> currents = new ArrayList<>();
        ArrayList<Double> rpms = new ArrayList<>();

        mMotorsToCheck = motorsToCheck;
        storeConfiguration();

        for(MotorConfig<T> config : motorsToCheck) {
            setMotorOutput(config.mMotor, 0.0);
        }

        int sensorPhaseMultiplier = 1;
      
        
        if(testerConfig.mRPMSupplier != null) {
            
            System.out.println("Turn mechanism attached to " + getMotorName(motorsToCheck.get(0).mMotor) + " forward NOW!");
            
            double position = 0;
            System.out.println("Starting test in:");
            for(int i = 4; i >= 0; i--) {
                for(int j = 0; j < 10; j++) {
                    Timer.delay(0.1);
                    System.out.print(".");
                    position += testerConfig.mRPMSupplier.getAsDouble() * 0.1;
                }
                System.out.println(i);
            }

            System.out.println("\n\n");

            if(position > 0) {
                System.out.println("Sensor Phase: FALSE");
                sensorPhaseMultiplier = 1;
            } else if(position < 0) {
                System.out.println("Sensor Phase: TRUE");
                sensorPhaseMultiplier = -1;
            } else {
                System.out.println("ERROR - Either mechanism was unturned or measuring device is not functional");
                sensorPhaseMultiplier = 1;
            }
  
        }

        for(MotorConfig<T> config : motorsToCheck) {
            System.out.println("\n\n");
            System.out.println("Testing: " + getMotorName(config.mMotor));

            setMotorOutput(config.mMotor, testerConfig.mOutputPercent);
            Timer.delay(testerConfig.mRuntime);

            double current = getMotorCurrent(config.mMotor);
            currents.add(current);

            if(testerConfig.mRPMSupplier != null) {
                double rpm = testerConfig.mRPMSupplier.getAsDouble() * sensorPhaseMultiplier;
                if(rpm > 0) {
                    rpms.add(rpm);
                    System.out.println(getMotorName(config.mMotor) + " is inverted: FALSE");
                } else if(rpm < 0) {
                    rpms.add(-rpm);
                    System.out.println(getMotorName(config.mMotor) + " is inverted: TRUE");
                } else {
                    System.out.println("Error - Malfunction in rotation measuring device");
                }
            }

            setMotorOutput(config.mMotor, 0.0);
            Timer.delay(testerConfig.mWaittime);   

        }

        System.out.println("\n\n");

        if(currents.size() > 0) {
            double average = currents.stream().mapToDouble(val -> val).average().getAsDouble();
            System.out.println("Average Current Draw: " + average);     
        }

        if(rpms.size() > 0) {
            double average = rpms.stream().mapToDouble(val -> val).average().getAsDouble();
            System.out.println("Average Speed: " + average);
        }

        System.out.println("\n\nTest Complete!");

        restoreConfiguration();
    }

    protected boolean checkMotorImpl(Subsystem subsystem, 
                                    ArrayList<MotorConfig<T>> motorsToCheck,
                                    CheckerConfig checkerConfig) {
        boolean failure = false;
        System.out.println("/////////////////////////////////////////");
        System.out.println("Checking Subsystem: " + subsystem.getClass() + 
            "for " + motorsToCheck.size() + " motors.");
        
        ArrayList<Double> currents = new ArrayList<>();
        ArrayList<Double> rpms = new ArrayList<>();

        mMotorsToCheck = motorsToCheck;
        storeConfiguration();

        for(MotorConfig<T> config : motorsToCheck) {
            setMotorOutput(config.mMotor, 0.0);
        }

        for(MotorConfig<T> config : motorsToCheck) {
            System.out.println("Checking: " + getMotorName(config.mMotor));

            setMotorOutput(config.mMotor, checkerConfig.mOutputPercent);
            Timer.delay(checkerConfig.mRuntime);

            double current = getMotorCurrent(config.mMotor);
            currents.add(current);
            System.out.print("Current: " + current);
            
            double rpm = Double.NaN;
            if(checkerConfig.mRPMSupplier != null) {
                rpm = checkerConfig.mRPMSupplier.getAsDouble();
                rpms.add(rpm);
                System.out.print(" RPM: " + rpm);
            }
            System.out.println();

            setMotorOutput(config.mMotor, 0.0);

            if(current < checkerConfig.mCurrentFloor) {
                System.out.println(getMotorName(config.mMotor) + " has failed current floor check vs " +
                    checkerConfig.mCurrentFloor + "!!");
                failure = true;
            }

            if(checkerConfig.mRPMSupplier != null) {
                if(rpm < checkerConfig.mRPMFloor) {
                    System.out.println(getMotorName(config.mMotor) + " has failed rpm floor check vs " +
                        checkerConfig.mRPMFloor + "!!");
                    failure = true;
                }
            }

            Timer.delay(checkerConfig.mWaittime);

        }


        if(currents.size() > 0) {
            double average = currents.stream().mapToDouble(val -> val).average().getAsDouble();

            if(!Util.allCloseTo(currents, average, checkerConfig.mCurrentEpsilon)) {
                System.out.println("Currents varied!!!!!!!");
                failure = true;
            }
        }

        if(rpms.size() > 0) {
            double average = currents.stream().mapToDouble(val -> val).average().getAsDouble();

            if(!Util.allCloseTo(rpms, average, checkerConfig.mRPMEpsilon)) {
                System.out.println("RPMs varied!!!!!!!");
                failure = true;
            }
        }

        restoreConfiguration();

        return !failure;
    }

    
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.MotorChecker;
import frc.lib.drivers.PheonixUtil;
import frc.lib.drivers.TalonFXChecker;
import frc.lib.drivers.TalonFXFactory;
import frc.lib.util.CircularBuffer;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.subsystems.requests.Request;


public class Shooter extends Subsystem {

    private static Shooter mInstance = null;

    public static Shooter getInstance() {
        if(mInstance == null) {
            mInstance = new Shooter();
        }
        return mInstance;
    }

    //debug
    private final boolean debug = false;

    //hardware
    private final LazyTalonFX mMasterShooter, mSlaveShooter;
    private final Limelight mLimelight = Limelight.getInstance();


    //controllers
    private final int kSpinUpSlot = 0;
    private final int kHoldSlot = 1;

    //states
    private ShooterControlState mControlState;
    private CircularBuffer mKfEstimator = new CircularBuffer(Constants.kShooterKfBufferSize);
    private boolean mOnTarget = false;
    private double mOnTargetStartTime = Double.POSITIVE_INFINITY;
    private PeriodicIO mPeriodicIO;
    private boolean mHasReadFromVision = false;
    private boolean mIsReadingFromVision = false;



    private void configAndEnableMotorLimits(LazyTalonFX falcon) {
        PheonixUtil.checkError(falcon.configVoltageCompSaturation(12.0, Constants.kTimeOutMs),
                falcon.getName() + " failed to set voltage compensation", false);
    
        falcon.enableVoltageCompensation(false);

        PheonixUtil.checkError(falcon.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 35, 40, 1), Constants.kTimeOutMs),
                falcon.getName() + " failed to set output current limit", false);

        PheonixUtil.checkError(falcon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 60, 0), Constants.kTimeOutMs),
                falcon.getName() + " failed to set input current limit", false);

    }

    private void disableMotorLimits(LazyTalonFX falcon) {
        falcon.enableVoltageCompensation(false);

        PheonixUtil.checkError(falcon.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 35, 40, 1), Constants.kTimeOutMs),
                falcon.getName() + " failed to disable output current limit", false);

        PheonixUtil.checkError(falcon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(false, 60, 60, 0), Constants.kTimeOutMs),
                falcon.getName() + " failed to disable input current limit", false);
    }


    private void configFalconForShooter(LazyTalonFX falcon, InvertType inversion) {
        falcon.setInverted(inversion);
        falcon.setNeutralMode(NeutralMode.Coast);
        configAndEnableMotorLimits(falcon);
    }


    private void configMasterForShooter(LazyTalonFX falcon, InvertType inversion, boolean sensorPhase) {
        configFalconForShooter(falcon, inversion);
        
        PheonixUtil.checkError(falcon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kTimeOutMs),
            falcon.getName() + " failed to set feedback sensor", true);
            
        falcon.setSensorPhase(sensorPhase);

        PheonixUtil.checkError(falcon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms, Constants.kTimeOutMs), 
                falcon.getName() + " failed to set velocity meas. period", true);
        
        PheonixUtil.checkError(falcon.configVelocityMeasurementWindow(1, Constants.kTimeOutMs), 
                falcon.getName() + " failed to set velocity meas. window", true);
        
        PheonixUtil.checkError(falcon.configOpenloopRamp(0.0, Constants.kTimeOutMs),
                falcon.getName() + " failed to set open loop ramp rate", true);

        PheonixUtil.checkError(falcon.configClosedloopRamp(0.0, Constants.kTimeOutMs),
                falcon.getName() + " failed to set closed loop ramp rate", true);
        
        PheonixUtil.checkError(falcon.configNeutralDeadband(0.0, Constants.kTimeOutMs), 
                falcon.getName() + " failed to set neutral deadband", true);   
    }

    /**
     * creates two pidf loops, one for spin up and one for holding (just kFF)
     */
    public synchronized void setControllerConstants() {
        mMasterShooter.config_kP(kSpinUpSlot, SmartDashboard.getNumber("Shooter kP", 0.0));
        mMasterShooter.config_kI(kSpinUpSlot, SmartDashboard.getNumber("Shooter kI", 0.0));
        mMasterShooter.config_kD(kSpinUpSlot, SmartDashboard.getNumber("Shooter kD", 0.0));
        mMasterShooter.config_kF(kSpinUpSlot, SmartDashboard.getNumber("Shooter kF", 0.031));
        mMasterShooter.config_IntegralZone(kSpinUpSlot, Constants.kShooterIZone);

        mMasterShooter.config_kP(kHoldSlot, 0.0);
        mMasterShooter.config_kI(kHoldSlot, 0.0);
        mMasterShooter.config_kD(kHoldSlot, 0.0);
        mMasterShooter.config_kF(kHoldSlot, Constants.kShooterHoldkF);
        mMasterShooter.config_IntegralZone(kHoldSlot, 0);
    }


    private Shooter() {
        mPeriodicIO = new PeriodicIO();

        mMasterShooter = TalonFXFactory.createDefaultFalcon("Left Shooter Motor", Ports.SHOOTER_LEFT_SHOOT_ID);
        configMasterForShooter(mMasterShooter, InvertType.InvertMotorOutput, false);


        mSlaveShooter = TalonFXFactory.createSlaveFalcon("Right Shooter Motor", Ports.SHOOTER_RIGHT_SHOOT_ID, Ports.SHOOTER_LEFT_SHOOT_ID);
        configFalconForShooter(mSlaveShooter, InvertType.OpposeMaster);
        mSlaveShooter.setMaster(mMasterShooter);

        setControllerConstants();

        SmartDashboard.putNumber("Shooter kP", 0.0);
        SmartDashboard.putNumber("Shooter kI", 0.000022);
        SmartDashboard.putNumber("Shooter kD", 0.0);
        SmartDashboard.putNumber("Shooter kF", 0.048000);

    }

    /**
     * remembers the inputs and outputs of the shooter
     */
    private static class PeriodicIO {
        //inputs
        public double velocity_in_ticks_per_100ms;
        public double output_percent;

        //outputs
        public double setpoint_rpm;
    }

    /**
     * reads velocity, voltage, and temperature
     */
    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.velocity_in_ticks_per_100ms = mMasterShooter.getSelectedSensorVelocity(0);
        mPeriodicIO.output_percent = mMasterShooter.getMotorOutputPercent(); 
    }



    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {

            @Override
            public void onStart(double timestamp) {
                synchronized(Shooter.this) {
                    mControlState = ShooterControlState.OPEN_LOOP;
                    mKfEstimator.clear();
                    mOnTarget = false;
                    mOnTargetStartTime = Double.POSITIVE_INFINITY;
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized(Shooter.this) {
                    if(mIsReadingFromVision) {
                        if(!mHasReadFromVision) {
                            if(mLimelight.seesTarget()) {
                                setHoldWhenReady(getRpmFromDistance(mLimelight.getDistance()));
                                mHasReadFromVision = true;
                            } else {
                                setHoldWhenReady(Constants.kStandardShootVelocity); //TODO: Change back
                            }
                        }
                    }

                    if(mControlState != ShooterControlState.OPEN_LOOP) {
                        handleClosedLoop(timestamp);
                    } else {
                        mKfEstimator.clear();
                        mOnTarget = false;
                        mOnTargetStartTime = Double.POSITIVE_INFINITY;
                    }
    
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
            
        });
    }



    private double getRpmFromDistance(double x) {
        return 4800;
    }

    public synchronized void shootUsingVision() {
        mIsReadingFromVision = true;
        mHasReadFromVision = false;
    }

    public synchronized void shootAtSetRpm(double rpm) {
        mIsReadingFromVision = false;
        setHoldWhenReady(rpm);
    }

    public synchronized boolean isShooterActive() {
        return mPeriodicIO.output_percent > 0.3;
    }

    /**
     * enum for managing the current status of the shooter.
     * open loop: for testing purposes
     * spin up: PIDF controller to reach desired RPM
     * hold when ready: brief state of calcuating kF required in hold stage
     * hold: state for mantaining rpm and quickly fixing any error when rapidly firing
     */
    public enum ShooterControlState {
        OPEN_LOOP, 
        SPIN_UP, 
        HOLD_WHEN_READY, 
        HOLD 
    }



    /**
     * configs for open loop if not in open loop.
     * sets current limit and voltage comp
     * @param percentOutput the output to set to the motor
     */
    public synchronized void setOpenLoop(double percentOutput) {
        if(mControlState != ShooterControlState.OPEN_LOOP) {
            mControlState = ShooterControlState.OPEN_LOOP;
            
            configAndEnableMotorLimits(mMasterShooter);
            configAndEnableMotorLimits(mSlaveShooter);
            mIsReadingFromVision = false;
        }
    
        mMasterShooter.set(ControlMode.PercentOutput, percentOutput);
    }

    /**
     * prepares for spin up and configures motors for spin up. 
     * @param setpointRpm what to set the target rpm
     */
    private void setSpinUp(double setpointRpm) {
        if(mControlState != ShooterControlState.SPIN_UP) {
            configForSpinUp();
        }
        mPeriodicIO.setpoint_rpm = setpointRpm * Constants.kRawVelocityToRpm;
    }

    /**
     * Configs motors for holding when ready.
     * @param setpointRpm target rpm for motor
     */
    private void setHoldWhenReady(double setpointRpm) {
        if(mControlState == ShooterControlState.SPIN_UP || mControlState == ShooterControlState.OPEN_LOOP) {
            configForHoldWhenReady();
        }
        mPeriodicIO.setpoint_rpm = setpointRpm * Constants.kRawVelocityToRpm;
    }

    /**
     * sets control state and changes pid slot
     * disables voltage comp and sets ramp rate
     */
    private void configForSpinUp() {
        mControlState = ShooterControlState.SPIN_UP;
        mMasterShooter.selectProfileSlot(kSpinUpSlot, 0);

        mMasterShooter.configClosedloopRamp(Constants.kShooterRampRate, Constants.kTimeOutMs);

        disableMotorLimits(mMasterShooter);
        disableMotorLimits(mSlaveShooter);
    }

    /**
     * Sets pid slod and control state.
     * Disables voltage comp and sets ramp rate.
     */
    private void configForHoldWhenReady() {
        mControlState = ShooterControlState.HOLD_WHEN_READY;
        mMasterShooter.selectProfileSlot(kSpinUpSlot, 0);

        mMasterShooter.configClosedloopRamp(Constants.kShooterRampRate, Constants.kTimeOutMs);

        disableMotorLimits(mMasterShooter);
        disableMotorLimits(mSlaveShooter);
    }

    /**
     * Sets pid slot and control state.
     * Sets feed forward using estimator and sets voltage comp
     */
    private void configForHold() {
        mControlState = ShooterControlState.HOLD;
        mMasterShooter.selectProfileSlot(kHoldSlot, 0);
        mMasterShooter.config_kF(kHoldSlot, 0.048);
        mMasterShooter.configClosedloopRamp(Constants.kShooterRampRate);

        disableMotorLimits(mMasterShooter);
        disableMotorLimits(mSlaveShooter);

        mMasterShooter.enableVoltageCompensation(true);
        mMasterShooter.enableVoltageCompensation(true);
    }

    /**
     * clears feed forward estimator and sets mOnTarget to false.
     */
    private void resetHold() {
        mKfEstimator.clear();
        mOnTarget = false;
    }

    /**
     * This method is the main controller of progressing the shooter through all the
     * stages (spin up -> hold when ready -> hold)
     * When the shooter is in the hold state, it is ready to fire
     * 
     * Get relatively precise when it comes to error with rpm (get close to wanted value), but once youre there, 
     * you have a bit more wiggle room with error from target rpm
     */
    private void handleClosedLoop(double timestamp) {
        if(mControlState == ShooterControlState.SPIN_UP) {
            mMasterShooter.set(ControlMode.Velocity, mPeriodicIO.setpoint_rpm);
            resetHold();
        } else if(mControlState == ShooterControlState.HOLD_WHEN_READY) {
            final double abs_error = Math.abs(mPeriodicIO.velocity_in_ticks_per_100ms - mPeriodicIO.setpoint_rpm);
            final boolean on_target_now = mOnTarget ? abs_error < Constants.kShooterStopOnTargetRpm :
                abs_error < Constants.kShooterStartOnTargetRpm;
            
            if(on_target_now && !mOnTarget) {
                mOnTargetStartTime = timestamp;
                mOnTarget = true;
            } else if(!on_target_now) {
                resetHold();
            }

            if(mOnTarget) {
                mKfEstimator.addValue(mPeriodicIO.output_percent);
            }

            if(mKfEstimator.getNumValues() >= Constants.kShooterMinOnTargetSamples) {
                configForHold();
            } else {
                mMasterShooter.set(ControlMode.Velocity, mPeriodicIO.setpoint_rpm);
            }

        }
    }

    /**
     * checks if the motor's rpm is on target using the fact that the shooter will only be in it's HOLD state when on target.
     * @return if motor rpm is on target
     */
    public synchronized boolean isOnTarget() {
        return mControlState == ShooterControlState.HOLD;
    }

    public synchronized boolean hasReachedSpinUpTarget() {
        return Math.abs(mPeriodicIO.setpoint_rpm - mPeriodicIO.velocity_in_ticks_per_100ms)
            < Constants.kShooterStartOnTargetRpm;
    }

    public synchronized double getCurrentRpm() {
        return mPeriodicIO.velocity_in_ticks_per_100ms / Constants.kRawVelocityToRpm;
    }

    /**
     * returns periodicIO's setpointrpm
     * @return setpoint rpm
     */
    public synchronized double getSetpointRpm() {
        return mPeriodicIO.setpoint_rpm;
    }


    public Request setVelocityAndWaitRequest(double velocity) {
        return new Request(){
        
            @Override
            public void act() {
                setHoldWhenReady(velocity);
            }

            @Override
            public boolean isFinished() {
                return isOnTarget();
            }
        };
    }

    public Request setVelocityRequest(double velocity) {
        return new Request() {

			@Override
			public void act() {
				setHoldWhenReady(velocity);
			}
        };
    }

    public Request setSpinUpAndWaitRequest(double velocity) {
        return new Request(){
        
            @Override
            public void act() {
                setSpinUp(velocity);
            }

            @Override
            public boolean isFinished() {
                return Math.abs(mPeriodicIO.velocity_in_ticks_per_100ms - mPeriodicIO.setpoint_rpm) < 1000;
            }
        };
    }

    public Request setSpinUpRequest(double velocity) {
        return new Request(){
        
            @Override
            public void act() {
                setSpinUp(velocity);
            }
        };
    }

    public Request setVelocityFromVisionRequest() {
        return new Request(){
            
            @Override
            public void act() {
                shootUsingVision();
            }

            @Override
            public boolean isFinished() {
                return isOnTarget();
            }
        };
    }

    /**
     * sets open loop to zero and rpm setpoint to zero
     */
    @Override
    public void stop() {
        setOpenLoop(0.0);
        mPeriodicIO.setpoint_rpm = 0.0;
    }

    /**
     * test method for the shooter
     */
    @Override
    public boolean checkSystem() {

        boolean check = TalonFXChecker.checkMotors(this,
            new ArrayList<MotorChecker.MotorConfig<LazyTalonFX>>() {    
            private static final long serialVersionUID = 1L;

                    {
                    add(new MotorChecker.MotorConfig<>(mMasterShooter));
                    add(new MotorChecker.MotorConfig<>(mSlaveShooter));
                }
            }, new MotorChecker.CheckerConfig() {
                {
                    mOutputPercent = 0.5;
                    mRuntime = 1.5;
                    mWaittime = 0.7;

                    mRPMFloor = 2000;
                    mRPMEpsilon = 5.0;

                    mRPMSupplier = () -> mMasterShooter.getSelectedSensorVelocity(0);
                }
            });

        return check;
    }


    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Ready to Fire?", isOnTarget());
        SmartDashboard.putNumber("Shooter Velocity RPM", getCurrentRpm());

        if(debug) {
            SmartDashboard.putString("Shooter state", mControlState.toString());

            SmartDashboard.putNumber("Shooter Velocity", mPeriodicIO.velocity_in_ticks_per_100ms);
            SmartDashboard.putNumber("Shooter Setpoint RPM", mPeriodicIO.setpoint_rpm);
            
            SmartDashboard.putNumber("Left Shooter Stator Current", mMasterShooter.getStatorCurrent());
            SmartDashboard.putNumber("Left Shooter Supply Current", mMasterShooter.getSupplyCurrent());
            SmartDashboard.putNumber("Left Shooter Voltage", mMasterShooter.getBusVoltage() * mMasterShooter.getMotorOutputPercent());

            SmartDashboard.putNumber("Right Shooter Stator Current", mSlaveShooter.getStatorCurrent());
            SmartDashboard.putNumber("Right Shooter Supply Current", mSlaveShooter.getSupplyCurrent());
            SmartDashboard.putNumber("Right Shooter Voltage", mSlaveShooter.getBusVoltage() * mSlaveShooter.getMotorOutputPercent());
        }
    }
}

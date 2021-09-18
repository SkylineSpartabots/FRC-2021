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

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.controllers.OverridesController;
import frc.lib.drivers.LazyTalonSRX;
import frc.lib.drivers.MotorChecker;
import frc.lib.drivers.PheonixUtil;
import frc.lib.drivers.TalonSRXChecker;
import frc.lib.drivers.TalonSRXFactory;
import frc.lib.drivers.TalonSRXUtil;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.subsystems.requests.Request;

public class Hopper extends Subsystem {

    private static Hopper mInstance = null;

    public static Hopper getInstance() {
        if (mInstance == null) {
            mInstance = new Hopper();
        }
        return mInstance;
    }

    // debug
    private final boolean debug = false;

    // hardware
    private final LazyTalonSRX mIndexMotor, mLeftBelt, mRightBelt;
    private final AnalogInput mIndexSensor;
    private final OverridesController mOverrides = OverridesController.getInstance();

    // control states
    private HopperControlState mCurrentState = HopperControlState.OFF;
    private boolean mStateChanged = false;
    private double mStateChangeTimestamp = 0.0;

    private double mIndexSensorBeganTimestamp = Double.POSITIVE_INFINITY;
    private double mLastSeenBallTimestamp = Double.POSITIVE_INFINITY;
    private double mStartUnjamTimstamp = Double.POSITIVE_INFINITY;
    private boolean mIsUnjamming = false;

    private boolean mHasBall = false;
    private int mNumberOfBallsShot = 0;

    private boolean mSlowIndex = false;

    private void configureBeltMotor(LazyTalonSRX talon, InvertType inversion) {
        talon.setInverted(inversion);

        PheonixUtil.checkError(talon.configVoltageCompSaturation(12.0, Constants.kTimeOutMs),
                talon.getName() + " failed to set voltage compensation", true);
        PheonixUtil.checkError(talon.configVoltageMeasurementFilter(32, Constants.kTimeOutMs),
                talon.getName() + " failed to set voltage meas. filter", true);
        talon.enableVoltageCompensation(true);

        TalonSRXUtil.setCurrentLimit(talon, 20);

        talon.setNeutralMode(NeutralMode.Coast);
    }

    private void configureIndexMotor(LazyTalonSRX talon, InvertType inversion) {
        talon.setInverted(inversion);

        PheonixUtil.checkError(talon.configVoltageCompSaturation(12.0, Constants.kTimeOutMs),
                talon.getName() + " failed to set voltage compensation", true);
        PheonixUtil.checkError(talon.configVoltageMeasurementFilter(32, Constants.kTimeOutMs),
                talon.getName() + " failed to set voltage meas. filter", true);
        talon.enableVoltageCompensation(true);

        TalonSRXUtil.setCurrentLimit(talon, 25);

        talon.setNeutralMode(NeutralMode.Brake);
    }

    private Hopper() {
        mIndexMotor = TalonSRXFactory.createDefaultTalon("Index Motor", Ports.HOPPER_INDEX_ID);
        configureIndexMotor(mIndexMotor, InvertType.None);

        mLeftBelt = TalonSRXFactory.createDefaultTalon("Left Belt Motor", Ports.HOPPER_LEFT_BELT);
        configureBeltMotor(mLeftBelt, InvertType.InvertMotorOutput);

        mRightBelt = TalonSRXFactory.createDefaultTalon("Right Belt Motor", Ports.HOPPER_RIGHT_BELT);
        configureBeltMotor(mRightBelt, InvertType.None);

        mIndexSensor = new AnalogInput(Ports.HOPPER_INDEX_SENSOR_PORT);
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {

            @Override
            public void onStart(double timestamp) {
                synchronized (Hopper.this) {
                    mHasBall = false;
                    mNumberOfBallsShot = 0;
                    mSlowIndex = true;
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Hopper.this) {
                    //System.out.println("Number of Balls: " + mNumberOfBallsShot);
                    if (mCurrentState == HopperControlState.SENSORED_INTAKE
                            || mCurrentState == HopperControlState.SENSORED_INDEX
                            || mCurrentState == HopperControlState.SMART_SENSORED_INDEX
                            || mCurrentState == HopperControlState.INDEX) {
                        if (rawBallDetected()) {
                            if (Double.isInfinite(mIndexSensorBeganTimestamp)) {
                                mIndexSensorBeganTimestamp = timestamp;
                            } else {
                                if (timestamp - mIndexSensorBeganTimestamp > 0.02) {
                                    mLastSeenBallTimestamp = timestamp;
                                    mHasBall = true;
                                }
                            }
                        } else if (!Double.isInfinite(mIndexSensorBeganTimestamp)) {
                            mIndexSensorBeganTimestamp = Double.POSITIVE_INFINITY;
                            mHasBall = false;
                            mNumberOfBallsShot++;
                        }
                    }

                    if (mCurrentState == HopperControlState.SENSORED_INTAKE) {
                        if (mHasBall) {
                            if(mSlowIndex) {
                                setBeltSpeed(0.25, 0.2);
                            } else {
                                setBeltSpeed(0.0, 0.0);
                            }
                            setIndexSpeed(0.0);
                        } else {
                            setIndexSpeed(mCurrentState.indexSpeed);
                            setBeltSpeed(mCurrentState.leftBeltSpeed, mCurrentState.rightBeltSpeed);
                        }
                    } else if (mCurrentState == HopperControlState.SENSORED_INDEX || mCurrentState == HopperControlState.SMART_SENSORED_INDEX) {
                        if(mCurrentState == HopperControlState.SMART_SENSORED_INDEX && !Drive.getInstance().hasAlginedToTarget()) {
                            setIndexSpeed(0.0);
                            setBeltSpeed(0.0, 0.0);
                        } else {
                            setIndexSpeed(mCurrentState.indexSpeed);
                            if (timestamp - mLastSeenBallTimestamp > Constants.kStartUnjamTimeThreshold) {
                                if (!mIsUnjamming) {
                                    mStartUnjamTimstamp = timestamp;
                                    mIsUnjamming = true;
                                }
                                if (timestamp - mStartUnjamTimstamp > Constants.kStopUnjamTime) {
                                    mLastSeenBallTimestamp = timestamp;
                                }
                                setBeltSpeed(mCurrentState.leftBeltSpeed, -mCurrentState.rightBeltSpeed);
                            } else {
                                setBeltSpeed(mCurrentState.leftBeltSpeed, mCurrentState.rightBeltSpeed);
                                mIsUnjamming = false;
                            }
                        } 
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {

            }

        });
    }

    public enum HopperControlState { 
        OFF(0.0, 0.0, 0.0), INDEX(0.7 , 0.4, 0.85), SENSORED_INDEX(0.5, 0.5, 0.85), SENSORED_INTAKE(0.2, 0.2, 0.2),
        SLOW_INDEX(0.2, 0.15, 0.0), REVERSE(-0.3, -0.5, -0.5), SMART_SENSORED_INDEX(0.8, 0.5, 0.75);

        public double indexSpeed = 0.0;
        public double leftBeltSpeed = 0.0;
        public double rightBeltSpeed = 0.0;

        private HopperControlState(double indexSpeed, double leftBeltSpeed, double rightBeltSpeed) {
            this.indexSpeed = indexSpeed;
            this.leftBeltSpeed = leftBeltSpeed;
            this.rightBeltSpeed = rightBeltSpeed;
        }
    }

    public synchronized void setSlowIndexState(boolean state) {
        mSlowIndex = state;
    }

    public synchronized HopperControlState getState() {
        return mCurrentState;
    }

    public synchronized void setState(HopperControlState newState) {
        if (newState != mCurrentState) {
            mStateChanged = true;
            mStateChangeTimestamp = Timer.getFPGATimestamp();
        }
        mCurrentState = newState;
    }

    public synchronized void setBeltSpeed(double leftBeltSpeed, double rightBeltSpeed) {
        mLeftBelt.set(ControlMode.PercentOutput, leftBeltSpeed);
        mRightBelt.set(ControlMode.PercentOutput, rightBeltSpeed);
    }

    public synchronized void setIndexSpeed(double indexSpeed) {
        mIndexMotor.set(ControlMode.PercentOutput, indexSpeed);
    }

    public synchronized void conformToState(HopperControlState desiredState) {
        setState(desiredState);
        if (desiredState != HopperControlState.SENSORED_INDEX && desiredState != HopperControlState.SENSORED_INTAKE
                && desiredState != HopperControlState.SMART_SENSORED_INDEX) {
            setBeltSpeed(desiredState.leftBeltSpeed, desiredState.rightBeltSpeed);
            setIndexSpeed(desiredState.indexSpeed);
        }
    }

    public synchronized boolean hasIndexedBall() {
        return mHasBall;
    }

    public synchronized double getNumberOfBallsShot() {
        return mNumberOfBallsShot;
    }

    private boolean rawBallDetected() {
        return mIndexSensor.getValue() > Constants.kIndexSensorThreshold;
    }

    public Request stateRequest(HopperControlState desiredState) {
        return new Request() {

            @Override
            public void act() {
                conformToState(desiredState);
            }
        };
    }

    public Request indexBallNumberRequest(double numberOfBalls) {
        return new Request() {
            double startTime = 0;
            double waitTime = 4;
            double startNumberOfBalls = mNumberOfBallsShot;

            @Override
            public void act() {
                startTime = Timer.getFPGATimestamp();
                conformToState(HopperControlState.SENSORED_INDEX);
            }

            @Override
            public boolean isFinished() {
                return (mNumberOfBallsShot >= startNumberOfBalls + numberOfBalls) 
                    || (Timer.getFPGATimestamp() - startTime) >= waitTime;
            }
        };
    }

    @Override
    public void stop() {
        conformToState(HopperControlState.OFF);
    }

    @Override
    public boolean checkSystem() {
        return TalonSRXChecker.checkMotors(this, new ArrayList<MotorChecker.MotorConfig<LazyTalonSRX>>() {
            private static final long serialVersionUID = 1L;

            {
                add(new MotorChecker.MotorConfig<>(mIndexMotor));
                add(new MotorChecker.MotorConfig<>(mLeftBelt));
                add(new MotorChecker.MotorConfig<>(mRightBelt));
            }
        }, new MotorChecker.CheckerConfig() {
            {
                mOutputPercent = 0.5;
                mRuntime = 1;
                mWaittime = 0.3;
                mRPMSupplier = null;
            }
        });
    }

    @Override
    public void outputTelemetry() {
        if (debug) {
            SmartDashboard.putString("Hopper State", mCurrentState.toString());

            SmartDashboard.putNumber("Index Supply Current", mIndexMotor.getSupplyCurrent());
            SmartDashboard.putNumber("Index Stator Current", mIndexMotor.getStatorCurrent());
            SmartDashboard.putNumber("Index Output", mIndexMotor.getLastSet());

            SmartDashboard.putNumber("Left Belt Supply Current", mLeftBelt.getSupplyCurrent());
            SmartDashboard.putNumber("Left Belt Stator Current", mLeftBelt.getStatorCurrent());
            SmartDashboard.putNumber("Left Belt Output", mLeftBelt.getLastSet());

            SmartDashboard.putNumber("Right Belt Supply Current", mRightBelt.getSupplyCurrent());
            SmartDashboard.putNumber("Right Belt Stator Current", mRightBelt.getStatorCurrent());
            SmartDashboard.putNumber("Right Belt Output", mRightBelt.getLastSet());
        }
    }
}

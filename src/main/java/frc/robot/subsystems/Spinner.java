/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;

import com.revrobotics.CANEncoder;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.LazySparkMax;
import frc.lib.drivers.MotorChecker;
import frc.lib.drivers.SparkMaxChecker;
import frc.lib.drivers.SparkMaxFactory;
import frc.lib.drivers.SparkMaxUtil;
import frc.lib.sensors.ColorSensor;
import frc.lib.sensors.ColorSensor.Colors;
import frc.lib.util.PIDController;
import frc.lib.util.TelemetryUtil;
import frc.lib.util.TelemetryUtil.PrintStyle;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.subsystems.requests.Prerequisite;
import frc.robot.subsystems.requests.Request;

/*
 * Add your docs here.
 */
public class Spinner extends Subsystem {

    private static Spinner mInstance = null;

    public static Spinner getInstance() {
        if(mInstance == null) {
            mInstance = new Spinner();
        }
        return mInstance;
    }

    //debug
    private final boolean debug = false;

    //hardware
    private final LazySparkMax mSpinnerMotor;
    private final ColorSensor mColorSensor;
    private final Solenoid mSpinnerSolenoid;
    private final CANEncoder mEncoder;

    //controllers
    private final PIDController mPidController;
    private double mEncoderTarget;
    private Colors mPositionColorTarget = Colors.UNKNOWN;
    private Colors mTurnToColorTarget = Colors.UNKNOWN;
    private Colors mPositionControlStartColor = Colors.UNKNOWN;
    private int mSpinDirectionMultiplier = 1;
    private boolean hasReachedColorTransition = false;

    //control states
    private SpinnerControlState mCurrentState;
    private boolean mStateChanged = false;
    private double mStateChangeTimestamp = 0;


    private void configureSparkForSpinner(LazySparkMax spark, boolean inversion) {
        spark.setInverted(inversion);
        SparkMaxUtil.checkError(spark.enableVoltageCompensation(12.0), spark.getName() + " failed to set voltage comp.", true);
        SparkMaxUtil.checkError(spark.setSmartCurrentLimit(20), spark.getName() + " failed to set current limit", true);
        SparkMaxUtil.checkError(spark.setOpenLoopRampRate(0.5), spark.getName() + " failed to set open loop ramp", true);
        SparkMaxUtil.checkError(spark.setClosedLoopRampRate(0.4), spark.getName() + " failed to set closed loop ramp", true);
    }


    private Spinner() {
        mSpinnerMotor = SparkMaxFactory.createDefaultSparkMax("Spinner Motor", Ports.SPINNER_MOTOR_ID, false);
        configureSparkForSpinner(mSpinnerMotor, false); //positive is spinning counter-clockwise

        mEncoder = mSpinnerMotor.getEncoder();

        mPidController = new PIDController(0.0, 0.0, 0.0);
        mPidController.setTolerance(25);
        mPidController.setMinMaxOutput(-1.0, 1.0);

        mSpinnerSolenoid = new Solenoid(Ports.PCM_ID, Ports.SPINNER_SOLENOID_ID);

        mColorSensor = ColorSensor.getInstance();
    }


    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {

            @Override
            public void onStart(double timestamp) {
                setOff();
            }

            @Override
            public void onLoop(double timestamp) {
                switch(mCurrentState) {
                    case OFF:
                        break;
                    case OPEN_LOOP:
                        break;
                    case ROTATION_CONTROL:
                        updateRotationControl();
                        break;
                    case POSITION_CONTROL:
                        updatePositionControl();
                        break;
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();

            }

        });
    }


    public enum SpinnerControlState {
        OFF,
        OPEN_LOOP,
        ROTATION_CONTROL,
        POSITION_CONTROL
    }


    public synchronized SpinnerControlState getState() {
        return mCurrentState;
    }

    private void setState(SpinnerControlState newState) {
        if(newState != mCurrentState) {
            mStateChanged = true;
            mStateChangeTimestamp = Timer.getFPGATimestamp();
        }
        mCurrentState = newState;
    }

    public synchronized void setOpenLoop(double percentOutput) {
        if(mCurrentState != SpinnerControlState.OPEN_LOOP) {
            mSpinnerSolenoid.set(true);
            setState(SpinnerControlState.OPEN_LOOP);
        }
        mSpinnerMotor.set(ControlType.kDutyCycle, percentOutput);
    }

    public synchronized void setOff() {
        if(mCurrentState != SpinnerControlState.OFF) {
            mSpinnerMotor.set(ControlType.kDutyCycle, 0.0);
            mSpinnerSolenoid.set(false);
            setState(SpinnerControlState.OFF);
        }
    }

    //rotation control uses bang-bang for speed, overshoot does not matter due to 3-5 rotation window
    public synchronized void setRotationControl() {
        if(mCurrentState != SpinnerControlState.ROTATION_CONTROL) {
            mSpinnerSolenoid.set(true);
            mEncoderTarget = mEncoder.getPosition() + (Constants.kCountsPerControlPanelRotation * 3.1);
            setState(SpinnerControlState.ROTATION_CONTROL);
        }
        mSpinnerMotor.set(ControlType.kDutyCycle, Constants.kRotationControlPercentOutput);
    }

    private void updateRotationControl() {
        if(mCurrentState == SpinnerControlState.ROTATION_CONTROL) {
            if(mEncoder.getPosition() > mEncoderTarget) {
                setOff();
            }
        } else {
            TelemetryUtil.print("Spinner is not in the rotation control state", PrintStyle.ERROR, true);
        }
    }

    public synchronized void setPositionControl() {
        if(mCurrentState != SpinnerControlState.POSITION_CONTROL) {
            getPositionControlColor();
            if(mTurnToColorTarget == Colors.UNKNOWN || !seesControlPanel()) {
                setOff();
                return;
            }
            mSpinnerSolenoid.set(true);
            mPositionControlStartColor = mColorSensor.getColor();
            hasReachedColorTransition = false;
            setState(SpinnerControlState.POSITION_CONTROL);
        }
    }

    private void updatePositionControl() {
        if(mCurrentState == SpinnerControlState.POSITION_CONTROL) {

            int currentColorOrdinal = mColorSensor.getColor().ordinal();

            if(!hasReachedColorTransition) {
                boolean isCurrentGreater = currentColorOrdinal > mPositionColorTarget.ordinal();
                int maxDiff = Math.max(currentColorOrdinal, mPositionColorTarget.ordinal()) - 
                    Math.min(currentColorOrdinal, mPositionColorTarget.ordinal());
                int minDiff = Math.min(currentColorOrdinal, mPositionColorTarget.ordinal()) + 4 
                    - Math.max(currentColorOrdinal, mPositionColorTarget.ordinal());
                
                mSpinDirectionMultiplier = isCurrentGreater && maxDiff > minDiff ? 1 : -1;

                if(currentColorOrdinal != mPositionControlStartColor.ordinal()) {
                    hasReachedColorTransition = true;
                    double spinTarget = (Math.min(maxDiff, minDiff) + 0.5) * (Constants.kCountsPerControlPanelRotation / 8);
                    mPidController.reset();
                    mPidController.setSetpoint(mEncoder.getPosition() + (spinTarget * mSpinDirectionMultiplier));
                } else {
                    mSpinnerMotor.set(ControlType.kDutyCycle, 0.3 * mSpinDirectionMultiplier);
                }
            } else {
                mSpinnerMotor.set(ControlType.kDutyCycle, mPidController.calculate(mEncoder.getPosition()));
            }

        } else {
            TelemetryUtil.print("Spinner is not in the position control state", PrintStyle.ERROR, true);
        }
    }

    public synchronized boolean seesControlPanel() {
        return mColorSensor.hasReachedPanel();
    }

    public synchronized void getPositionControlColor() {
        String gameData = DriverStation.getInstance().getGameSpecificMessage();
        if(gameData.length() > 0) {
            switch(gameData.charAt(0)) {
                case 'B':
                    mPositionColorTarget = Colors.BLUE;
                    mTurnToColorTarget = Colors.RED;
                    break;
                case 'G':
                    mPositionColorTarget = Colors.GREEN;
                    mTurnToColorTarget = Colors.YELLOW;
                    break;
                case 'R':
                    mPositionColorTarget = Colors.RED;
                    mTurnToColorTarget = Colors.BLUE;
                    break;
                case 'Y':
                    mPositionColorTarget = Colors.YELLOW;
                    mTurnToColorTarget = Colors.GREEN;
                    break;
                default:
                    mPositionColorTarget = Colors.UNKNOWN;
                    TelemetryUtil.print("Corrupt game data recieved", PrintStyle.ERROR, true);
            }
        } else {
            mPositionColorTarget = Colors.UNKNOWN;
            
            TelemetryUtil.print("NO GAME DATA RECIEVED", PrintStyle.ERROR, true);
        }

        SmartDashboard.putString("Spinner Color:", mPositionColorTarget.toString());
    }

    
    public Request openLopoRequest(double percentOutput) {
        return new Request(){
        
            @Override
            public void act() {
                setOpenLoop(percentOutput);
            }
        };
    }

    public Request rotationControlRequest() {  
        Request request = new Request(){
            @Override
            public void act() {
                setRotationControl();
            }

            @Override
            public boolean isFinished() {
                return !allowed();
            }
        };

        request.withPrerequisite(canSpinPrerequisite);

        return request;
    }

    public Request positionControlRequest() {
        Request request = new Request(){
        
            @Override
            public void act() {
                setPositionControl();
            }

            @Override
            public boolean isFinished() {
                return !allowed();
            }
        };

        request.withPrerequisite(canSpinPrerequisite);
       
       return request;
    }

    public Prerequisite deployedPrerequisite = new Prerequisite(){
        @Override
        public boolean met() {
            return mSpinnerSolenoid.get();
        }
    };

    public Prerequisite canSpinPrerequisite = new Prerequisite(){
        @Override
        public boolean met() {
            return mSpinnerSolenoid.get() && mColorSensor.hasReachedPanel();
        }
    };


    @Override
    public void stop() {
        setOff();
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        if(debug) {
            


        }
    }
}

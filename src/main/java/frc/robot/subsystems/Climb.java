/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.LazyTalonSRX;
import frc.lib.drivers.PheonixUtil;
import frc.lib.drivers.TalonFXFactory;
import frc.lib.drivers.TalonFXUtil;
import frc.lib.drivers.TalonSRXFactory;
import frc.lib.drivers.TalonSRXUtil;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.subsystems.requests.Request;


public class Climb extends Subsystem {

    private static Climb mInstance = null;

    public static Climb getInstance() {
        if(mInstance == null) {
            mInstance = new Climb();
        }
        return mInstance;
     }


     //hardware
     private final LazyTalonSRX mHookSlideMotor;
     private final LazyTalonFX mMasterWinch, mSlaveWinch;
     private final Encoder mClimbHookEncoder;
     

     //hardware states
     private ClimbControlState mCurrentState;
     private boolean mStateChanged = false;
     private double mStateChangeTimestamp = 0.0;


    private void configureHookMotor(InvertType inversion) {
        mHookSlideMotor.setInverted(inversion);

        /*PheonixUtil.checkError(mHookSlideMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
            0, Constants.kTimeOutMs), "Hook Slide Motor failed to configure feedback device", true);
        mHookSlideMotor.setSensorPhase(false);*/
        
        //mHookSlideMotor.setSelectedSensorPosition(0);

        /*PheonixUtil.checkError(mHookSlideMotor.configForwardSoftLimitThreshold(10000, Constants.kTimeOutMs),
             "Hook Slide Motor failed to set forward soft limit threshold", true);
        mHookSlideMotor.configForwardSoftLimitEnable(false);

        PheonixUtil.checkError(mHookSlideMotor.configReverseSoftLimitThreshold(0, Constants.kTimeOutMs),
             "Hook Slide Motor failed to set reverse soft limit threshold", true);
        mHookSlideMotor.configReverseSoftLimitEnable(false);*/

        PheonixUtil.checkError(mHookSlideMotor.configVoltageCompSaturation(12.0, Constants.kTimeOutMs), 
             "Hook Slide Motor failed tp set voltage compensation", true);
        mHookSlideMotor.enableVoltageCompensation(true);

        TalonSRXUtil.setCurrentLimit(mHookSlideMotor, 5);

        mHookSlideMotor.setNeutralMode(NeutralMode.Brake);
    }

    private void configureWinchMotor(LazyTalonFX falcon, InvertType inversion) {
        falcon.setInverted(inversion);

        TalonFXUtil.setStatorCurrentLimit(falcon, 40);
        TalonFXUtil.setSupplyCurrentLimit(falcon, 60);

        PheonixUtil.checkError(falcon.configVoltageCompSaturation(12.0, Constants.kTimeOutMs), 
             falcon.getName() + " failed tp set voltage compensation", true);
        mHookSlideMotor.enableVoltageCompensation(true);

        falcon.setNeutralMode(NeutralMode.Coast);
    }

    private Climb() {
        mHookSlideMotor = TalonSRXFactory.createDefaultTalon("Hook Slide Motor", Ports.CLIMB_HOOK_ID);
        configureHookMotor(InvertType.None);
    
        mMasterWinch = TalonFXFactory.createDefaultFalcon("Master Winch Motor", Ports.CLIMB_MASTER_WINCH_ID);
        configureWinchMotor(mMasterWinch, InvertType.None);

        mSlaveWinch = TalonFXFactory.createSlaveFalcon("Slave Winch Motor", Ports.CLIMB_SLAVE_WINCH_ID, Ports.CLIMB_MASTER_WINCH_ID);
        mSlaveWinch.setMaster(mMasterWinch);
        configureWinchMotor(mSlaveWinch, InvertType.OpposeMaster);

        mClimbHookEncoder = new Encoder(Ports.CLIMB_ENCODER_B, Ports.CLIMB_ENCODER_A);
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {

            @Override
            public void onStart(double timestamp) {
                mClimbHookEncoder.reset();
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized(Climb.this) {
                    if(mCurrentState == ClimbControlState.RAISE_HOOK) {
                        if(mClimbHookEncoder.getDistance() > Constants.kClimbMaxHeight) {
                            mHookSlideMotor.set(ControlMode.PercentOutput, 0.0);
                        }
                    } else if(mCurrentState == ClimbControlState.LOWER_HOOK) {
                        if(mClimbHookEncoder.getDistance() < 0.0) {
                            mHookSlideMotor.set(ControlMode.PercentOutput, 0.0);
                        }
                    }
                }
                
            }

            @Override
            public void onStop(double timestamp) {
                stop();

            }

        });
    }

    public enum ClimbControlState {
        OFF(0.0, 0.0),
        RAISE_HOOK(0.35, 0.0),
        LOWER_HOOK(-0.2, 0.0),
        OVERRIDE_RAISE_HOOK(0.25, 0.0),
        OVERRIDE_LOWER_HOOK(-0.2, 0.0),
        WINCH_UP(0.0, 0.95),
        AUTO_WINCH_UP(-0.4, 0.95);

        private double hookSlideSpeed = 0.0;
        private double winchSpeed = 0.0;

        private ClimbControlState(double hookSlideSpeed, double winchSpeed) {
            this.hookSlideSpeed = hookSlideSpeed;
            this.winchSpeed = winchSpeed;
        }
    }

    public synchronized ClimbControlState getState() {
        return mCurrentState;
    }

    public synchronized void setState(ClimbControlState newState) {
        if(newState != mCurrentState) {
            mStateChanged = true;
            mStateChangeTimestamp = Timer.getFPGATimestamp();
        }
        mCurrentState = newState;
    }

    public synchronized void setHookSlideSpeed(double percentOutput) {
        mHookSlideMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    public synchronized void setWinchSpeed(double percentOutput) {
        mMasterWinch.set(ControlMode.PercentOutput, Math.abs(percentOutput));
    }

    public synchronized void conformToState(ClimbControlState desiredState) {
        setState(desiredState);


        if(mCurrentState == ClimbControlState.RAISE_HOOK && mClimbHookEncoder.getDistance() > Constants.kClimbMaxHeight) {
            setHookSlideSpeed(0.0);
        } else if(mCurrentState == ClimbControlState.LOWER_HOOK && mClimbHookEncoder.getDistance() < 0.0) {
            setHookSlideSpeed(0.0);
        } else {
            setHookSlideSpeed(desiredState.hookSlideSpeed);
        }
        
        setWinchSpeed(desiredState.winchSpeed);
    }

    public Request stateRequest(ClimbControlState desiredState) {
        return new Request(){
        
            @Override
            public void act() {
                conformToState(desiredState);
            }
        };
    }


    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
    
    }
}

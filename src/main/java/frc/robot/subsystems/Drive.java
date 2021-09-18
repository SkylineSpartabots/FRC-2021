
package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.MotorChecker;
import frc.lib.drivers.PheonixUtil;
import frc.lib.drivers.TalonFXChecker;
import frc.lib.drivers.TalonFXFactory;
import frc.lib.drivers.TalonFXUtil;
import frc.lib.sensors.Navx;
import frc.lib.util.DriveSignal;
import frc.lib.util.PIDController;
import frc.lib.util.TelemetryUtil;
import frc.lib.util.Util;
import frc.lib.util.TelemetryUtil.PrintStyle;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.subsystems.requests.Request;

public class Drive extends Subsystem {

    private static Drive mInstance = null;

    public static Drive getInstance() {
        if (mInstance == null) {
            mInstance = new Drive();
        }
        return mInstance;
    }

    // debug
    private final boolean debug = false;

    // hardware
    private final LazyTalonFX mLeftMaster, mLeftSlave, mRightMaster, mRightSlave;
    private final Navx mNavx;
    private final Limelight mLimelight = Limelight.getInstance();

    // hardware states
    private boolean mIsBrakeMode;
    private Rotation2d mGyroOffset;
    private PeriodicIO mPeriodicIO;
    private final DifferentialDriveOdometry mOdometry;

    // controllers
    private final RamseteController mRamseteController;
    private final PIDController mLeftPidController, mRightPidController, mTurnPidController, mAlignPidController;
    
    private boolean mIsAlignedToTarget = false;
    private boolean mRawIsAlignedToTarget = false;
    private double mBeganAlignedToTarget = Double.POSITIVE_INFINITY;

    private final SimpleMotorFeedforward mFeedforwardController;
    private final DifferentialDriveKinematics mDriveKinematics;
    private DifferentialDriveWheelSpeeds mPrevWheelSpeeds;
    private Trajectory mCurrentPath = null;
    private double mTimeSincePathStart = 0.0;
    private double mPrevPathClockCycleTime = 0.0;
    private double mRememberedGyroTarget = 0.0;

    // control states
    private DriveControlState mDriveControlState;

    /**
     * sets the motor's inversion, open loop ramp, closed loop ramp, voltage comp
     * sat, and current limits
     * 
     * @param falcon    the falcon being set up
     * @param inversion inverted state
     */
    private synchronized void configureMotorForDrive(LazyTalonFX falcon, InvertType inversion) {
        falcon.setInverted(inversion);

        PheonixUtil.checkError(falcon.configVoltageCompSaturation(12.0, Constants.kTimeOutMs),
                falcon.getName() + " failed to set voltage compensation", true);

        falcon.enableVoltageCompensation(true);

        PheonixUtil.checkError(falcon.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 35, 35, 0)),
                falcon.getName() + " failed to set output current limit", true);

        PheonixUtil.checkError(falcon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 60, 0)),
                falcon.getName() + " failed to set input current limit", true);

    }

    /**
     * configures the sensor phase and feedback coefficient for a master motor
     * 
     * @param falcon      the falcon being set up
     * @param inversion   inverted state
     * @param sensorPhase is sensor in same direction as motor
     */
    private synchronized void configureMasterForDrive(LazyTalonFX falcon, InvertType inversion, boolean sensorPhase) {
        configureMotorForDrive(falcon, inversion);
        PheonixUtil.checkError(
                falcon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kTimeOutMs),
                falcon.getName() + " failed to set feedback sensor", true);

        falcon.setSensorPhase(sensorPhase);

        PheonixUtil.checkError(
                falcon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kTimeOutMs),
                falcon.getName() + " failed to set velocity meas. period", true);

        PheonixUtil.checkError(falcon.configVelocityMeasurementWindow(1, Constants.kTimeOutMs),
                falcon.getName() + " failed to set velocity meas. window", true);

        PheonixUtil.checkError(falcon.configOpenloopRamp(0.3, Constants.kTimeOutMs),
                falcon.getName() + " failed to set open loop ramp rate", true);

        PheonixUtil.checkError(falcon.configClosedloopRamp(0.0, Constants.kTimeOutMs),
                falcon.getName() + " failed to set closed loop ramp rate", true);

        PheonixUtil.checkError(falcon.configNeutralDeadband(0.0, Constants.kTimeOutMs),
                falcon.getName() + " failed to set neutral deadband", true);
    }

    private Drive() {
        mPeriodicIO = new PeriodicIO();

        mLeftMaster = TalonFXFactory.createDefaultFalcon("Drive Left Master", Ports.DRIVE_LEFT_MASTER_ID);
        configureMasterForDrive(mLeftMaster, InvertType.None, false);

        mLeftSlave = TalonFXFactory.createSlaveFalcon("Drive Left Slave", Ports.DRIVE_LEFT_SLAVE_ID,
                Ports.DRIVE_LEFT_MASTER_ID);
        configureMotorForDrive(mLeftSlave, InvertType.FollowMaster);
        mLeftSlave.setMaster(mLeftMaster);

        mRightMaster = TalonFXFactory.createDefaultFalcon("Drive Right Master", Ports.DRIVE_RIGHT_MASTER_ID);
        configureMasterForDrive(mRightMaster, InvertType.InvertMotorOutput, false);

        mRightSlave = TalonFXFactory.createSlaveFalcon("Drive Right Slave", Ports.DRIVE_RIGHT_SLAVE_ID,
                Ports.DRIVE_RIGHT_MASTER_ID);
        configureMotorForDrive(mRightSlave, InvertType.FollowMaster);
        mRightSlave.setMaster(mRightMaster);

        mNavx = Navx.getInstance();
        mNavx.reset();

        mOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(mNavx.getHeading()));
        mRamseteController = new RamseteController();
        mDriveKinematics = new DifferentialDriveKinematics(Constants.kDriveWheelTrackWidthMeters);

        mFeedforwardController = new SimpleMotorFeedforward(Constants.kDriveKsVolts, Constants.kDriveKvVolts,
                Constants.kDriveKaVolts);
        mLeftPidController = new PIDController(Constants.kDrivePathingProportion, 0.0, 0.0);
        mRightPidController = new PIDController(Constants.kDrivePathingProportion, 0.0, 0.0);

        mTurnPidController = new PIDController(0.0029, 0.0, 0.0);
        mTurnPidController.enableContinuousInput(-180, 180);
        mTurnPidController.setTolerance(1.5);
        mTurnPidController.setMinMaxOutput(-0.7, 0.7);

        mAlignPidController = new PIDController(0.0022, 0.0001, 0.00);
        mAlignPidController.setIntegratorRange(-0.1, 0.1);
        mAlignPidController.enableContinuousInput(-180, 180);
        mAlignPidController.setTolerance(1.5);
        mAlignPidController.setMinMaxOutput(-0.7, 0.7);

        mIsBrakeMode = true;
        setBrakeMode(false);
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    /**
     * tracks the inputs and outputs of the drive train
     */
    private static class PeriodicIO {
        // inputs
        public double timestamp;

        public double left_position; // meters
        public double right_position;

        public double left_velocity; // meters per second
        public double right_velocity;

        public Rotation2d heading = new Rotation2d();

        // outputs
        public double left_demand;
        public double right_demand;
    }

    /**
     * reads encoder position, change in position, velocity, heading, and
     * temperature
     */
    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        mPeriodicIO.left_position = (mLeftMaster.getSelectedSensorPosition() / 2048.0) * 0.0972 * Math.PI
                * Constants.kDriveWheelDiameterInMeters;

        mPeriodicIO.right_position = (mRightMaster.getSelectedSensorPosition() / 2048.0) * 0.0972 * Math.PI
                * Constants.kDriveWheelDiameterInMeters;
        mPeriodicIO.left_velocity = (mLeftMaster.getSelectedSensorVelocity() / 2048) * 0.0972 * Math.PI
                * Constants.kDriveWheelDiameterInMeters * 20.0;

        mPeriodicIO.left_velocity = (mRightMaster.getSelectedSensorVelocity() / 2048) * 0.0972 * Math.PI
                * Constants.kDriveWheelDiameterInMeters * 20.0;

        mPeriodicIO.heading = Rotation2d.fromDegrees(mNavx.getHeading()).minus(mGyroOffset);

        mOdometry.update(mPeriodicIO.heading, mPeriodicIO.left_position, mPeriodicIO.right_position);

    }

    /**
     * if open loop: sets output from periodic io if path following: sets desired
     * velocity to motors from periodic io arbitrary feed forward adds the fourth
     * parameter to the output
     */
    @Override
    public void writePeriodicOutputs() {
        if (mDriveControlState != null) {
            mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.left_demand);
            mRightMaster.set(ControlMode.PercentOutput, mPeriodicIO.right_demand);
        }
    }

    /**
     * registers drive train's loop to subsystem manager
     */
    @Override
    public void registerEnabledLoops(ILooper loop) {
        loop.register(new Loop() {

            @Override
            public void onStart(double timestamp) {
                synchronized (Drive.this) {
                    stop();
                    setBrakeMode(false);
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Drive.this) {
                    handleFaults();

                    switch (mDriveControlState) {
                    case OPEN_LOOP:
                        break;
                    case PATH_FOLLOWING:
                        if (mCurrentPath != null) {
                            updatePathFollower();
                        }
                        break;
                    case TURN_PID:
                        updateTurnPid();
                        break;
                    case ALIGN_TO_TARGET:
                        updateAlignController();
                        break;
                    default:
                        TelemetryUtil.print("Drive in an unexpected control state", PrintStyle.ERROR, false);
                        break;
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                synchronized(Drive.this) {
                    stop();
                    setBrakeMode(false);
                }
                
            }
        });
    }

    public synchronized Pose2d getDrivePose() {
        return mOdometry.getPoseMeters();
    }

    public synchronized void resetOdometry(Pose2d pose) {
        resetEncoders();
        mOdometry.resetPosition(pose, getHeading());
    }

    public double getLeftPosition() {
        return mPeriodicIO.left_position;
    }

    public double getRightPosition() {
        return mPeriodicIO.right_position;
    }

    public double getLeftLinearVelocity() {
        return mPeriodicIO.left_velocity;
    }

    public double getRightLinearVelocity() {
        return mPeriodicIO.right_velocity;
    }

    public double getLinearVelocity() {
        return (getRightLinearVelocity() + getLeftLinearVelocity()) / 2.0;
    }

    public double getAverageDriveVelocityMagnitude() {
        return (Math.abs(getRightLinearVelocity()) + Math.abs(getLeftLinearVelocity())) / 2.0;
    }

    public double getAngularVelocity() {
        return (getRightLinearVelocity() - getLeftLinearVelocity()) / Constants.kDriveWheelTrackWidthMeters;
    }

    /**
     * 
     * Sets a drive signal to the motors. If the robot is not in a driving state, it
     * will turn break mode on.
     * 
     * @param signal drive signal to set percent power to motors
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            setBrakeMode(true);
            PheonixUtil.checkError(mLeftMaster.configNeutralDeadband(0.04, Constants.kTimeOutMs),
                    mLeftMaster.getName() + " failed to set neutral deadband on openloop transition", true);
            PheonixUtil.checkError(mRightMaster.configNeutralDeadband(0.04, Constants.kTimeOutMs),
                    mRightMaster.getName() + " failed to set neutral deadband on openloop transition", true);
            mCurrentPath = null;
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }

        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
    }

    private double mQuickStopAccumulator;

    /**
     * sets power to the robot in such a way to create a curved drive
     * 
     * @param throttle  forward amount
     * @param curve     turning amount
     * @param quickTurn should the robot turn quickly
     */
    public synchronized void setCurvatureDrive(double throttle, double curve, boolean quickTurn) {
        throttle = Util.limit(throttle, -1.0, 1.0);
        throttle = Util.deadBand(throttle, 0.04);

        curve = Util.limit(curve, -1.0, 1.0);
        curve = Util.deadBand(curve, 0.04);

        double angularPower;
        boolean overPower;

        if (quickTurn) {
            if (Math.abs(throttle) < Constants.kQuickStopThreshold) {
                mQuickStopAccumulator = (1 - Constants.kQuickStopAlpha) * mQuickStopAccumulator
                        + Constants.kQuickStopAlpha * Util.limit(curve, -1.0, 1.0) * 2;
            }
            overPower = true;
            angularPower = curve;
        } else {
            overPower = false;
            angularPower = Math.abs(throttle) * curve - mQuickStopAccumulator;

            if (mQuickStopAccumulator > 1) {
                mQuickStopAccumulator -= 1;
            } else if (mQuickStopAccumulator < -1) {
                mQuickStopAccumulator += 1;
            } else {
                mQuickStopAccumulator = 0.0;
            }
        }

        double leftMotorOutput = throttle + angularPower;
        double rightMotorOutput = throttle - angularPower;

        // If rotation is overpowered, reduce both outputs to within acceptable range
        if (overPower) {
            if (leftMotorOutput > 1.0) {
                rightMotorOutput -= leftMotorOutput - 1.0;
                leftMotorOutput = 1.0;
            } else if (rightMotorOutput > 1.0) {
                leftMotorOutput -= rightMotorOutput - 1.0;
                rightMotorOutput = 1.0;
            } else if (leftMotorOutput < -1.0) {
                rightMotorOutput -= leftMotorOutput + 1.0;
                leftMotorOutput = -1.0;
            } else if (rightMotorOutput < -1.0) {
                leftMotorOutput -= rightMotorOutput + 1.0;
                rightMotorOutput = -1.0;
            }
        }

        // Normalize the wheel speeds
        double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
        if (maxMagnitude > 1.0) {
            leftMotorOutput /= maxMagnitude;
            rightMotorOutput /= maxMagnitude;
        }

        setOpenLoop(new DriveSignal(leftMotorOutput, rightMotorOutput));
    }

    /**
     * the turning algorithm is the same as the one used in DifferentialDrive.
     * 
     * @param throttle forward amount
     * @param turn     turning amount
     */
    public synchronized void setArcadeDrive(double throttle, double turn) {
        throttle = Util.limit(throttle, -1, 1);
        throttle = Util.deadBand(throttle, 0.04);

        turn = Util.limit(turn, -1, 1);
        turn = Util.deadBand(turn, 0.04);

        throttle = Math.copySign(throttle * throttle, throttle);
        turn = Math.copySign(turn * turn, turn);

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(throttle), Math.abs(turn)), throttle);

        if (throttle >= 0.0) {
            if (turn >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = throttle - turn;
            } else {
                leftMotorOutput = throttle + turn;
                rightMotorOutput = maxInput;
            }
        } else {
            if (turn >= 0.0) {
                leftMotorOutput = throttle + turn;
                rightMotorOutput = maxInput;
            } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = throttle - turn;
            }
        }

        SmartDashboard.putNumber("Right Power", rightMotorOutput);
        setOpenLoop(new DriveSignal(Util.limit(leftMotorOutput, -1.0, 1.0), Util.limit(rightMotorOutput, -1.0, 1.0)));

    }

    /**
     * sets velocity demand during path following mode. if not in path following
     * mode, turns on break mode and limits stator current.
     * 
     * @param signal      the drive signal to set motor velocity
     * @param feedforward the drive signal to set motor feedforward
     */
    public synchronized void setVelocity(DriveSignal signal) {
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            setBrakeMode(true);
            setStatorCurrentLimit(35);
            PheonixUtil.checkError(mLeftMaster.configNeutralDeadband(0.0, Constants.kTimeOutMs),
                    mLeftMaster.getName() + " failed to set neutral deadband on pathing transition", true);
            PheonixUtil.checkError(mRightMaster.configNeutralDeadband(0.0, Constants.kTimeOutMs),
                    mRightMaster.getName() + " failed to set neutral deadband on pathing transition", true);
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }

        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
    }

    public synchronized void setDrivePath(Trajectory path) {
        if (mCurrentPath != path || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            mCurrentPath = path;
            

            Trajectory.State initialState = path.sample(0);
            mPrevWheelSpeeds = mDriveKinematics.toWheelSpeeds(new ChassisSpeeds(initialState.velocityMetersPerSecond, 0,
                    initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));

            mLeftPidController.reset();
            mRightPidController.reset();

            mTimeSincePathStart = Timer.getFPGATimestamp();
            mPrevPathClockCycleTime = Timer.getFPGATimestamp();

            if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
                setBrakeMode(true);
                setStatorCurrentLimit(35);
                PheonixUtil.checkError(mLeftMaster.configNeutralDeadband(0.0, Constants.kTimeOutMs),
                        mLeftMaster.getName() + " failed to set neutral deadband on pathing transition", true);
                PheonixUtil.checkError(mRightMaster.configNeutralDeadband(0.0, Constants.kTimeOutMs),
                        mRightMaster.getName() + " failed to set neutral deadband on pathing transition", true);
                mDriveControlState = DriveControlState.PATH_FOLLOWING;
            }
        }
    }

    public synchronized boolean isDoneWithPath() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mCurrentPath != null) {
            return (Timer.getFPGATimestamp() - mTimeSincePathStart) > mCurrentPath.getTotalTimeSeconds();
        }
        return true;
    }

    private void updatePathFollower() {
        if (mCurrentPath != null && mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            double currentTime = Timer.getFPGATimestamp();
            double dt = currentTime - mPrevPathClockCycleTime;

            DifferentialDriveWheelSpeeds wheelSpeeds = mDriveKinematics.toWheelSpeeds(mRamseteController
                    .calculate(getDrivePose(), mCurrentPath.sample(currentTime - mTimeSincePathStart)));

            double leftVelocitySetpoint = wheelSpeeds.leftMetersPerSecond;
            double rightVelocitySetpoint = wheelSpeeds.rightMetersPerSecond;

            double leftOutput = mFeedforwardController.calculate(leftVelocitySetpoint,
                    (leftVelocitySetpoint - mPrevWheelSpeeds.leftMetersPerSecond) / dt);

            double rightOutput = mFeedforwardController.calculate(rightVelocitySetpoint,
                    (rightVelocitySetpoint - mPrevWheelSpeeds.rightMetersPerSecond) / dt);

            leftOutput += mLeftPidController.calculate(getLeftLinearVelocity(), leftVelocitySetpoint);
            rightOutput += mRightPidController.calculate(getRightLinearVelocity(), rightVelocitySetpoint);

            setVelocity(new DriveSignal(leftOutput, rightOutput));

            mPrevPathClockCycleTime = currentTime;
            mPrevWheelSpeeds = wheelSpeeds;

        } else {
            TelemetryUtil.print("Robot is not in a path following state", PrintStyle.ERROR, true);
        }

    }

    public synchronized void turnToHeading(double desiredHeading, boolean reset) {
        if (mDriveControlState != DriveControlState.TURN_PID) {
            setBrakeMode(true);
            setStatorCurrentLimit(35);
            PheonixUtil.checkError(mLeftMaster.configNeutralDeadband(0.0, Constants.kTimeOutMs),
                    mLeftMaster.getName() + " failed to set neutral deadband on pathing transition", true);
            PheonixUtil.checkError(mRightMaster.configNeutralDeadband(0.0, Constants.kTimeOutMs),
                    mRightMaster.getName() + " failed to set neutral deadband on pathing transition", true);
            mCurrentPath = null;
            mDriveControlState = DriveControlState.TURN_PID;
        }

        if (reset) {
            mTurnPidController.reset();
        }

        mTurnPidController.setSetpoint(desiredHeading);
    }

    private void updateTurnPid() {
        if (mDriveControlState == DriveControlState.TURN_PID) {
            double output = mAlignPidController.calculate(getHeading().getDegrees());
            if(Math.abs(mAlignPidController.getPositionError()) > 0.5) {
                output += Constants.kDriveTurnKs * Math.signum(output);
            }
            setVelocity(new DriveSignal(output, -output));
        } else {
            TelemetryUtil.print("Robot is not in a turn pid state", PrintStyle.ERROR, true);
        }
    }

    public synchronized boolean hasReachedHeadingTarget() {
        if (mDriveControlState == DriveControlState.TURN_PID) {
            return mTurnPidController.atSetpoint();
        }
        return false;
    }

    public synchronized void setAlignToTarget() {
        if (mDriveControlState != DriveControlState.ALIGN_TO_TARGET && Math.abs(getHeading().getDegrees()) < 45.0) {
            setBrakeMode(true);
            setStatorCurrentLimit(35);
            PheonixUtil.checkError(mLeftMaster.configNeutralDeadband(0.0, Constants.kTimeOutMs),
                    mLeftMaster.getName() + " failed to set neutral deadband on pathing transition", true);
            PheonixUtil.checkError(mRightMaster.configNeutralDeadband(0.0, Constants.kTimeOutMs),
                    mRightMaster.getName() + " failed to set neutral deadband on pathing transition", true);
            
            mIsAlignedToTarget = false;
            mRawIsAlignedToTarget = false;
            mBeganAlignedToTarget = Double.POSITIVE_INFINITY;

            mAlignPidController.reset();
            mRememberedGyroTarget = 0.0;
            mCurrentPath = null;
            mDriveControlState = DriveControlState.ALIGN_TO_TARGET;
            

        }
    }

    private void updateAlignController() {
        if (mDriveControlState == DriveControlState.ALIGN_TO_TARGET) {
            if (mLimelight.seesTarget()) { 
                mRememberedGyroTarget = getHeading().getDegrees() + mLimelight.getXOffset();
                
                if(mAlignPidController.atSetpoint()) {
                    if(!mRawIsAlignedToTarget) {
                        mBeganAlignedToTarget = Timer.getFPGATimestamp();
                    } else {
                        if(Timer.getFPGATimestamp() - mBeganAlignedToTarget > 0.2) {
                            mIsAlignedToTarget = true;
                        }
                    }
                    mRawIsAlignedToTarget = true;

                } else {
                    mRawIsAlignedToTarget = false;
                    mIsAlignedToTarget = false;
                    mBeganAlignedToTarget = Double.POSITIVE_INFINITY;
                }

            } else {
                mIsAlignedToTarget = false;
            }

            double output = mAlignPidController.calculate(getHeading().getDegrees(), mRememberedGyroTarget);
            if(Math.abs(mAlignPidController.getPositionError()) > 0.5) {
                output += 0.03 * Math.signum(output);
            }
            setVelocity(new DriveSignal(output, -output));

        } else {
            TelemetryUtil.print("Robot is not in a target aligning state", PrintStyle.ERROR, true);
        }
    }

    public synchronized boolean hasAlginedToTarget() {
        if (mDriveControlState == DriveControlState.ALIGN_TO_TARGET) {
            return mIsAlignedToTarget;
        }

        return false;
    }

    /**
     * Gets the heading of the robot (using navx)
     * 
     * @return a Rotation2d object representing the current direction of the robot
     */
    public synchronized Rotation2d getHeading() {
        return mPeriodicIO.heading;
    }

    public synchronized void resetHeading() {
        mGyroOffset = Rotation2d.fromDegrees(mNavx.getHeading());
        mPeriodicIO.heading = Rotation2d.fromDegrees(mNavx.getHeading()).minus(mGyroOffset);
    }

    /**
     * Sets left and right encoder cound to zero.
     */
    public synchronized void resetEncoders() {
        PheonixUtil.checkError(mLeftMaster.setSelectedSensorPosition(0, 0, Constants.kTimeOutMs),
                mLeftMaster.getName() + " failed to reset encoder", true);
        PheonixUtil.checkError(mRightMaster.setSelectedSensorPosition(0, 0, Constants.kTimeOutMs),
                mRightMaster.getName() + " failed to reset encoder", true);
    }

    /**
     * Sets all motors to the specified brake mode. And updates inner break mode
     * state.
     * 
     * @param enableBrake if true sets break mode, if false sets to neutral mode.
     */
    public synchronized void setBrakeMode(boolean enableBrake) {
        if (enableBrake != mIsBrakeMode) {
            TelemetryUtil.print("Is Brake Mode: " + enableBrake, PrintStyle.ERROR, true);
            NeutralMode mode = enableBrake ? NeutralMode.Brake : NeutralMode.Coast;
            mLeftMaster.setNeutralMode(mode);
            mLeftSlave.setNeutralMode(mode);
            mRightMaster.setNeutralMode(mode);
            mRightSlave.setNeutralMode(mode);
            mIsBrakeMode = enableBrake;
        }
    }

    /**
     * Sets all motors' stator current to specified amps
     * 
     * @param amps stator limit to set all motors to in amps
     */
    public synchronized void setStatorCurrentLimit(int amps) {
        TalonFXUtil.setStatorCurrentLimit(mLeftMaster, amps);
        TalonFXUtil.setStatorCurrentLimit(mLeftSlave, amps);
        TalonFXUtil.setStatorCurrentLimit(mRightMaster, amps);
        TalonFXUtil.setStatorCurrentLimit(mRightSlave, amps);
    }

    /**
     * checks all talons for sticky faults, and prints errors as needed (done in
     * .checkSensorFaults()) if faults are present, they are cleared
     */
    public synchronized void handleFaults() {
        TalonFXUtil.checkSensorFaults(mLeftMaster);
        TalonFXUtil.checkSlaveFaults(mLeftSlave, Ports.DRIVE_LEFT_MASTER_ID);
        TalonFXUtil.checkSensorFaults(mRightMaster);
        TalonFXUtil.checkSlaveFaults(mRightSlave, Ports.DRIVE_RIGHT_MASTER_ID);
    }

    /**
     * enum to keep track of drive control status
     */
    private enum DriveControlState {
        OPEN_LOOP, PATH_FOLLOWING, TURN_PID, ALIGN_TO_TARGET;
    }

    /**
     * resets heading and encoder
     */
    @Override
    public void zeroSensors() {
        mPeriodicIO = new PeriodicIO();
        mNavx.reset();
        resetHeading();
        resetOdometry(new Pose2d());
    }

    /**
     * stops the drivetrain. Sets open loop to neutral
     */
    @Override
    public void stop() {
        setVelocity(DriveSignal.NEUTRAL);
    }

    /**
     * Outputs telemetry messages to .dsevents or smartdashboard
     * 
     * Outputs if the drive train is overheating
     */
    @Override
    public void outputTelemetry() {
        /*
        SmartDashboard.putNumber("NavX Heading", getHeading().getDegrees());
        SmartDashboard.putNumber("Odometry X", mOdometry.getPoseMeters().getTranslation().getX());
        SmartDashboard.putNumber("Odometry Y", mOdometry.getPoseMeters().getTranslation().getY());
        SmartDashboard.putNumber("Odometry Heading", mOdometry.getPoseMeters().getRotation().getDegrees());*/
        
        SmartDashboard.putBoolean("Is Aligned To Target", hasAlginedToTarget());
        
    }

    public Request openLoopRequest(DriveSignal driveSignal) {
        return new Request() {

            @Override
            public void act() {
                setOpenLoop(driveSignal);
            }
        };
    }

    public Request timeDriveRequest(DriveSignal driveSignal, double time) {
        return new Request() {
            double startTime = 0;

            @Override
            public void act() {
                startTime = Timer.getFPGATimestamp();
                setOpenLoop(driveSignal);
            }

            @Override
            public boolean isFinished() {
                return (Timer.getFPGATimestamp() - startTime) >= time;
            }
        };
    }

    public Request turnRequest(double heading, double timeout) {
        return new Request() {
            double startTime = 0;

            @Override
            public void act() {
                startTime = Timer.getFPGATimestamp();
                turnToHeading(heading, true);
            }

            @Override
            public boolean isFinished() {
                return (Timer.getFPGATimestamp() - startTime) >= timeout || hasReachedHeadingTarget();
            }
        };
    }

    public Request alignToTargetRequest() {
        return new Request() {

            @Override
            public void act() {
                setAlignToTarget();
            }

            @Override
            public boolean isFinished() {
                return hasAlginedToTarget();
            }
        };
    }

    @Override
    public boolean checkSystem() {
        setBrakeMode(false);

        boolean leftSide = TalonFXChecker.checkMotors(this, new ArrayList<MotorChecker.MotorConfig<LazyTalonFX>>() {

            private static final long serialVersionUID = 1L;

            {
                add(new MotorChecker.MotorConfig<LazyTalonFX>(mLeftMaster));
                add(new MotorChecker.MotorConfig<LazyTalonFX>(mLeftSlave));
            }
        }, new MotorChecker.CheckerConfig() {
            {
                mCurrentFloor = 4;
                mRPMFloor = 0.3;
                mCurrentEpsilon = 2.0;
                mRPMEpsilon = 2;
                mRPMSupplier = () -> getLeftPosition();
                mRuntime = 1.0;
                mWaittime = 0.3;
            }
        });

        boolean rightSide = TalonFXChecker.checkMotors(this, new ArrayList<MotorChecker.MotorConfig<LazyTalonFX>>() {

            private static final long serialVersionUID = 1L;

            {
                add(new MotorChecker.MotorConfig<LazyTalonFX>(mRightMaster));
                add(new MotorChecker.MotorConfig<LazyTalonFX>(mRightSlave));
            }
        }, new MotorChecker.CheckerConfig() {
            {
                mCurrentFloor = 4;
                mRPMFloor = 0.3;
                mCurrentEpsilon = 2.0;
                mRPMEpsilon = 2;
                mRPMSupplier = () -> getRightPosition();
                mRuntime = 1.0;
                mWaittime = 0.3;
            }
        });

        return leftSide && rightSide;
    }

}
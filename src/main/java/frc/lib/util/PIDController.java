
package frc.lib.util;

import edu.wpi.first.wpilibj.Timer;

public class PIDController {
    private double mKp;
    private double mKi;
    private double mKd;

    private double mMaximumIntegral = 1.0;
    private double mMinimumIntegral = -1.0;

    private double mMaximumInput;
    private double mMinimumInput;

    private double mMaximumOutput = 1.0;
    private double mMinimumOutput = -1.0;

    private double mInputRange;

    private boolean mIsWrapping;

    private double mPrevTimestamp = 0.0;

    private double mPositionError;
    private double mVelocityError;

    private double mPrevError;
    private double mTotalError;

    private double mPositionTolerance = 0.05;
    private double mVelocityTolerance = Double.POSITIVE_INFINITY;

    private double mSetpoint;

    public PIDController(double Kp, double Ki, double Kd) {
        mKp = Kp;
        mKi = Ki;
        mKd = Kd;
    }

    public void setPID(double Kp, double Ki, double Kd) {
        mKp = Kp;
        mKi = Ki;
        mKd = Kd;
    }

    public void setP(double Kp) {
        mKp = Kp;
    }

    public void setI(double Ki) {
        mKi = Ki;
    }

    public void setD(double Kd) {
        mKd = Kd;
    }

    public double getP() {
        return mKp;
    }

    public double getI() {
        return mKi;
    }

    public double getD() {
        return mKd;
    }

    public void setMinMaxOutput(double min, double max) {
        mMinimumOutput = min;
        mMaximumOutput = max;
    }

    public void setSetpoint(double setpoint) {
        if (mMaximumInput > mMinimumInput) {
            mSetpoint = Util.limit(setpoint, mMinimumInput, mMaximumInput);
        } else {
            mSetpoint = setpoint;
        }
    }

    public double getSetpoint() {
        return mSetpoint;
    }

    public boolean atSetpoint() {
        return Math.abs(mPositionError) < mPositionTolerance && Math.abs(mVelocityError) < mVelocityTolerance;
    }

    public void enableContinuousInput(double minimumInput, double maximumInput) {
        mIsWrapping = true;
        setInputRange(minimumInput, maximumInput);
    }

    public void disableContinuousInput() {
        mIsWrapping = false;
    }

    public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
        mMinimumIntegral = minimumIntegral;
        mMaximumIntegral = maximumIntegral;
    }

    public void setTolerance(double positionTolerance) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }

    public void setTolerance(double positionTolerance, double velocityTolerance) {
        mPositionTolerance = positionTolerance;
        mVelocityTolerance = velocityTolerance;
    }

    public double getPositionError() {
        return getContinuousError(mPositionError);
    }

    public double getVelocityError() {
        return mVelocityError;
    }

    public double calculate(double measurement, double setpoint) {
        setSetpoint(setpoint);
        return calculate(measurement);
    }

    public double calculate(double measurement) {
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - mPrevTimestamp;

        mPrevError = mPositionError;
        mPositionError = getContinuousError(mSetpoint - measurement);
        mVelocityError = (mPositionError - mPrevError) / dt;

        if (mKi != 0) {
            mTotalError = Util.limit(mTotalError + mPositionError * dt, mMinimumIntegral / mKi,
                    mMaximumIntegral / mKi);
        }

        mPrevTimestamp = currentTime;
        double output = mKp * mPositionError + mKi * mTotalError + mKd * mVelocityError;
        return Util.limit(output, mMinimumOutput, mMaximumOutput);
    }

    public void reset() {
        mPrevError = 0;
        mTotalError = 0;
        mPrevTimestamp = Timer.getFPGATimestamp();
    }

    protected double getContinuousError(double error) {
        if (mIsWrapping && mInputRange > 0) {
            error %= mInputRange;
            if (Math.abs(error) > mInputRange / 2) {
                if (error > 0) {
                    return error - mInputRange;
                } else {
                    return error + mInputRange;
                }
            }
        }
        return error;
    }

    private void setInputRange(double minimumInput, double maximumInput) {
        mMinimumInput = minimumInput;
        mMaximumInput = maximumInput;
        mInputRange = maximumInput - minimumInput;

        if (mMaximumInput > mMinimumInput) {
            mSetpoint = Util.limit(mSetpoint, mMinimumInput, mMaximumInput);
        }
    }

}

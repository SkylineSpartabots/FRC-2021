/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.actions.tests;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.DriveSignal;
import frc.lib.util.TelemetryUtil;
import frc.lib.util.TelemetryUtil.PrintStyle;
import frc.robot.auto.actions.Action;
import frc.robot.subsystems.Drive;


public class FindDriveAccelerationAction implements Action {
    private final double mPower;
    private final double mTotalTime;
    private final boolean mTurn;
    private final boolean mReverse;

    private static final Drive mDrive = Drive.getInstance();

    private double mMaxAcceleration = 0.0;

    private double mStartTime = 0.0;
    private double mPrevTime = 0.0;
    private double mPrevVelocity = 0.0;

    public FindDriveAccelerationAction(double power, double time, boolean reverse, boolean turn) {
        mPower = power;
        mTotalTime = time;
        mReverse = reverse;
        mTurn = turn;
    }

    @Override
    public void start() {
        mDrive.setOpenLoop(new DriveSignal((mReverse ? -1.0 : 1.0) * mPower, 
            (mReverse ? -1.0 : 1.0) * (mTurn ? -1.0 : 1.0) * mPower));
        mStartTime = Timer.getFPGATimestamp();
        mPrevTime = mStartTime;
    }

    @Override
    public void update() {
        double currentVelocity = mDrive.getAverageDriveVelocityMagnitude();
        double currentTime = Timer.getFPGATimestamp();

        if (mPrevTime == mStartTime) {
            mPrevTime = currentTime;
            mPrevVelocity = currentVelocity;
            return;
        }

        double acceleration = (currentVelocity - mPrevVelocity) / (currentTime - mPrevTime);

        if(acceleration > mMaxAcceleration) {
            mMaxAcceleration = acceleration;
        }

        mPrevTime = currentTime;
        mPrevVelocity = currentVelocity;
    }

    @Override
    public boolean isFinished() {
        
        return Timer.getFPGATimestamp() - mStartTime > mTotalTime;
    }

    @Override
    public void done() {
        mDrive.setOpenLoop(DriveSignal.BRAKE);
        TelemetryUtil.print("Maximum Drive Acceleration: " + mMaxAcceleration, PrintStyle.INFO, true);
    }



    
}

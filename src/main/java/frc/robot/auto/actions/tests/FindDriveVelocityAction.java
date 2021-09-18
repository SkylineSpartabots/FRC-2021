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


public class FindDriveVelocityAction implements Action {

    private static final double kRampRate = 0.02;
    private double maxVelocity = 0.0;

    private final double mMaxPower;
    private final boolean mTurn;
    private final boolean mReverse;

    private static final Drive mDrive = Drive.getInstance();

    private boolean isFinished = false;
    private double mStartTime = 0.0;

    public FindDriveVelocityAction(double maxPower, boolean reverse, boolean turn) {
        mMaxPower = maxPower;
        mReverse = reverse;
        mTurn = turn;
    }


    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {
        double percentOutput = kRampRate * (Timer.getFPGATimestamp() - mStartTime);
        if(percentOutput > mMaxPower) {
            isFinished = true;
            return;
        }

        mDrive.setOpenLoop(new DriveSignal((mReverse ? -1.0 : 1.0) * percentOutput, 
            (mReverse ? -1.0 : 1.0) * (mTurn ? -1.0 : 1.0) * percentOutput));

        maxVelocity = mDrive.getAverageDriveVelocityMagnitude();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void done() {
        mDrive.setOpenLoop(DriveSignal.BRAKE);
        TelemetryUtil.print("Maximum Drive Velocity: " + maxVelocity, PrintStyle.INFO, true);
    }
}

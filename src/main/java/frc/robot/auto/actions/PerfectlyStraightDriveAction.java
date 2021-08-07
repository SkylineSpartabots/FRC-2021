/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.lib.util.DriveSignal;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

/**
 * Add your docs here.
 */
public class PerfectlyStraightDriveAction implements Action {


    
    private double mLeftDistance, mRightDistance;
    private double mPower;
    private Rotation2d mDesiredGyroHeading;
    private double mDistanceInMeters;

    private Drive mDrive = Drive.getInstance();


    /**
     * 
     * @param heading heading
     * @param distanceInMeters distance
     * @param power power
     */
    public PerfectlyStraightDriveAction(Rotation2d heading, double distanceInMeters, double power) {
        mDesiredGyroHeading = heading;
        mDistanceInMeters = distanceInMeters;
        mPower = power;

        mRightDistance = mLeftDistance = power > 0 ? distanceInMeters : -distanceInMeters;
    }


    @Override
    public void start() {
        mRightDistance += mDrive.getRightPosition();
        mLeftDistance += mDrive.getLeftPosition();
    }

    @Override
    public void update() {
        double turnCorrection = (mDrive.getHeading().getDegrees() - mDesiredGyroHeading.getDegrees()) 
            * Constants.kPerfectlyDriveProportion;
        mDrive.setOpenLoop(new DriveSignal(mPower - turnCorrection, mPower + turnCorrection));
    }

    @Override
    public boolean isFinished() {
        if(mPower > 0) {
            return mRightDistance < mDrive.getRightPosition() && mLeftDistance < mDrive.getLeftPosition();
        }
        
        return mRightDistance > mDrive.getRightPosition() && mLeftDistance > mDrive.getLeftPosition();
    }

    @Override
    public void done() {

    }


}

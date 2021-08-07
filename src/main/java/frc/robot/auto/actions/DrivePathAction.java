/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.lib.util.DriveSignal;
import frc.robot.subsystems.Drive;

/**
 * Add your docs here.
 */
public class DrivePathAction implements Action {

    private Trajectory mPath;
    private Drive mDrive = Drive.getInstance();
    private boolean mStopWhenDone;

    public DrivePathAction(Trajectory path, boolean stopWhenDone) {
        mPath = path;
        mStopWhenDone = stopWhenDone;
    }

    
    @Override
    public void start() {
        mDrive.setDrivePath(mPath);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return mDrive.isDoneWithPath();
    }

    @Override
    public void done() {
        if(mStopWhenDone) {
            mDrive.setBrakeMode(true);
            mDrive.setVelocity(new DriveSignal(0, 0));
        }
    }
}

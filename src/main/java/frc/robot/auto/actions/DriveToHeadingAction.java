/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.actions;

import frc.lib.util.DriveSignal;
import frc.robot.subsystems.Drive;

/**
 * Add your docs here.
 */
public class DriveToHeadingAction implements Action {

    private final double mDesiredHeading;

    public DriveToHeadingAction(double heading) {
        mDesiredHeading = heading;
    }


    @Override
    public void start() {
        Drive.getInstance().turnToHeading(mDesiredHeading, true);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return Drive.getInstance().hasReachedHeadingTarget();
    }

    @Override
    public void done() {
        Drive.getInstance().setOpenLoop(new DriveSignal(0.0, 0.0));

    }
}

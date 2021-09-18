/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.actions;

import frc.lib.util.DriveSignal;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Limelight;

/**
 * Add your docs here.
 */
public class TurnUntilSeeTargetAction implements Action {

    private boolean mIsLeft;
    public TurnUntilSeeTargetAction(boolean isLeft) {
        mIsLeft = isLeft;
    }

    
    @Override
    public void start() {
        Drive.getInstance().setOpenLoop(mIsLeft ? new DriveSignal(-0.5, 0.5) : new DriveSignal(0.5, -0.5));
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return Limelight.getInstance().seesTarget();
    }

    @Override
    public void done() {
        Drive.getInstance().setOpenLoop(new DriveSignal(0.0, 0.0));
    }
}

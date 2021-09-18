/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.modes;

import frc.robot.AutoModeSelector.StartingPosition;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.DriveOpenLoopAction;
import frc.robot.auto.actions.WaitForRequestsAction;
import frc.robot.auto.actions.WaitAction;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Hopper.HopperControlState;

/**
 * Add your docs here.
 */
public class ShootThree extends AutoModeBase {

    private final StartingPosition mStartPosition;
    private final boolean mCrossLine;

    public ShootThree(StartingPosition startingPosition, boolean crossLine) {
        mStartPosition = startingPosition;
        mCrossLine = crossLine;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        Shooter.getInstance().setOpenLoop(0.0);
        
        if(mStartPosition == StartingPosition.TARGET_SIDE) {
            Superstructure.getInstance().autoShootBalls(3);
            runAction(new WaitForRequestsAction());
            runAction(new WaitAction(0.2));
            Hopper.getInstance().conformToState(HopperControlState.OFF);
            Shooter.getInstance().setOpenLoop(0.0);
            if(mCrossLine) {
                runAction(new DriveOpenLoopAction(0.5, 0.5, 0.5));
            }
            
        }

    }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.modes;

import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.DriveOpenLoopAction;

/**
 * Add your docs here.
 */
public class CrossLine extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new DriveOpenLoopAction(-0.3, -0.3, 1.5));
    }
}

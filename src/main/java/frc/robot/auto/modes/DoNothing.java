/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.modes;

import frc.lib.util.TelemetryUtil;
import frc.lib.util.TelemetryUtil.PrintStyle;
import frc.robot.auto.AutoModeEndedException;

/**
 * Add your docs here.
 */
public class DoNothing extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        TelemetryUtil.print("Big woop, I am doing absolutely nothing", PrintStyle.INFO, true);
    }
}

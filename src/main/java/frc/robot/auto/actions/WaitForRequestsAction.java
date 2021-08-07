/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.actions;

import frc.robot.subsystems.Superstructure;

/**
 * Add your docs here.
 */
public class WaitForRequestsAction implements Action {

    @Override
    public boolean isFinished() {
        return Superstructure.getInstance().requestsCompleted();
    }
    
    @Override
    public void start() {

    }

    @Override
    public void update() {

    }


    @Override
    public void done() {

    }
}

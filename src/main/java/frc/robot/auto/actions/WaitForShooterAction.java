/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.actions;

import frc.robot.subsystems.Shooter;

/**
 * Add your docs here.
 */
public class WaitForShooterAction implements Action {

    @Override
    public boolean isFinished() {
        return Shooter.getInstance().isOnTarget();
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

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;

/**
 * Add your docs here.
 */
public class WaitThenLambdaAction implements Action {

    private LambdaSupplier mF;
    private double mWaitTime;

    public WaitThenLambdaAction(LambdaSupplier f, double waitTime) {
        mF = f;
        mWaitTime = waitTime;
    }

    @Override
    public void start() {
        mWaitTime += Timer.getFPGATimestamp();
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return mWaitTime > Timer.getFPGATimestamp();
    }

    @Override
    public void done() {
        mF.f();
    }
}

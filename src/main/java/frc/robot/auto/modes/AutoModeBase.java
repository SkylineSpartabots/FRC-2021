/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.modes;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.Action;
import frc.robot.paths.PathGenerator;

public abstract class AutoModeBase {
    protected final double mUpdateRate = 1.0 / 50.0;
    protected boolean mActive = false;
    protected boolean mIsInterrupted = false;

    protected static PathGenerator.PathSet paths = PathGenerator.getInstance().getPathSet();

    protected abstract void routine() throws AutoModeEndedException;

    public void run() {
        mActive = true;

        try {
            routine();
        } catch (AutoModeEndedException e) {
            DriverStation.reportError("AUTO MODE DONE!!!! ENDED EARLY!!!!", false);
            return;
        }

        done();
    }

    public void done() {
        System.out.println("Auto mode done");
    }

    public void stop() {
        mActive = false;
    }

    public boolean isActive() {
        return mActive;
    }

    public boolean isActiveWithThrow() throws AutoModeEndedException {
        if (!isActive()) {
            throw new AutoModeEndedException();
        }

        return isActive();
    }

   
    public void interrupt() {
        System.out.println("** Auto mode interrrupted!");
        mIsInterrupted = true;
    }

    public void resume() {
        System.out.println("** Auto mode resumed!");
        mIsInterrupted = false;
    }

    public void runAction(Action action) throws AutoModeEndedException {
        isActiveWithThrow();
        long waitTime = (long) (mUpdateRate * 1000.0);

        // Wait for interrupt state to clear
        while (isActiveWithThrow() && mIsInterrupted) {
            try {
                Thread.sleep(waitTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        action.start();

        // Run action, stop action on interrupt, non active mode, or done
        while (isActiveWithThrow() && !action.isFinished() && !mIsInterrupted) {
            action.update();

            try {
                Thread.sleep(waitTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        action.done();

    }

    public boolean getIsInterrupted() {
        return mIsInterrupted;
    }
}
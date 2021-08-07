/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.actions;

import java.util.ArrayList;
import java.util.List;

/**
 * Add your docs here.
 */
public class SeriesAction implements Action {

    private Action mCurrentAction;
    private final ArrayList<Action> mRemainingActions;

    public SeriesAction(List<Action> actions) {
        mRemainingActions = new ArrayList<>(actions);
        mCurrentAction = null;
    }

    @Override
    public void start() {}

    @Override
    public void update() {
        if(mCurrentAction == null) {
            if(mRemainingActions.isEmpty()) {
                return;
            }

            mCurrentAction = mRemainingActions.remove(0);
            mCurrentAction.start();
        }

        mCurrentAction.update();

        if(mCurrentAction.isFinished()) {
            mCurrentAction.done();
            mCurrentAction = null;
        }
    }

    @Override
    public boolean isFinished() {
        return mRemainingActions.isEmpty() && mCurrentAction == null;
    }

    @Override
    public void done() {}
}

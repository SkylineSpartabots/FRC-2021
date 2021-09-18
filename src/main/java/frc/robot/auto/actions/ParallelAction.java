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
public class ParallelAction implements Action {

    private final ArrayList<Action> mActions;
    
    public ParallelAction(List<Action> actions) {
        mActions = new ArrayList<>(actions);
    }
    @Override
    public void start() {
        for(Action action : mActions) {
            action.start();
        }
    }

    @Override
    public void update() {
        for(Action action : mActions) {
            action.update();
        }
    }

    @Override
    public boolean isFinished() {
        for(Action action : mActions) {
            if(!action.isFinished()) {
                return false;
            }
        }

        return true;
    }

    @Override
    public void done() {
        for(Action action : mActions) {
            action.done();
        }
    }
}

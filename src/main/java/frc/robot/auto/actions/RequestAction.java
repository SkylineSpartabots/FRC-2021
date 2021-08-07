/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.actions;

import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.requests.RequestList;

/**
 * Add your docs here.
 */
public class RequestAction implements Action {


    private RequestList mRequestList;
    private RequestList mQueuedList;

    public RequestAction(RequestList requestList) {
        this(requestList, null);
    }

    public RequestAction(RequestList requestList, RequestList queuedList) {
        mRequestList = requestList;
        mQueuedList = queuedList;
    }


    @Override
    public void start() {
        Superstructure.getInstance().request(mRequestList);

        if(mQueuedList != null) {
            Superstructure.getInstance().replaceQueue(mQueuedList);
        }
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return Superstructure.getInstance().requestsCompleted();
    }

    @Override
    public void done() {

    }
}

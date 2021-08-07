/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.DriveSignal;
import frc.lib.util.TelemetryUtil;
import frc.lib.util.TelemetryUtil.PrintStyle;
import frc.robot.Constants;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.subsystems.Climb.ClimbControlState;
import frc.robot.subsystems.Hopper.HopperControlState;
import frc.robot.subsystems.Intake.IntakeControlState;
import frc.robot.subsystems.requests.Request;
import frc.robot.subsystems.requests.RequestList;

/**
 * Add your docs here.
 */
public class Superstructure extends Subsystem {

    private static Superstructure mInstance = null;

    public static Superstructure getInstance() {
        if(mInstance == null) {
            mInstance = new Superstructure();
        }
        return mInstance;
    }

    private Drive mDrive;
    private Shooter mShooter;
   // private Spinner mSpinner = Spinner.getInstance();
    private Hopper mHopper;
    private Intake mIntake;
    private Limelight mLimelight;
    private Climb mClimb;

    private RequestList activeRequests;
    private ArrayList<RequestList> queuedRequests;
    private Request currentRequest;

    private boolean newRequests = false;
    private boolean activeRequestsCompleted = false;
    private boolean allRequestsCompleted = false;


    private Superstructure() {
        mDrive = Drive.getInstance();
        mShooter = Shooter.getInstance();
        mHopper = Hopper.getInstance();
        mIntake = Intake.getInstance();
        mClimb = Climb.getInstance();
        mLimelight = Limelight.getInstance();

        queuedRequests = new ArrayList<>(0);
    }

    public boolean requestsCompleted() {
        return allRequestsCompleted;
    }

    private void setQueuedRequests(RequestList requests) {
        queuedRequests.clear();
        queuedRequests.add(requests);
    }

    private void setQueuedRequests(List<RequestList> requests) {
        queuedRequests.clear();
        queuedRequests = new ArrayList<>(requests.size());
        for (RequestList list : requests) {
            queuedRequests.add(list);
        }
    }

    private void setActiveRequests(RequestList requests) {
        activeRequests = requests;
        newRequests = true;
        activeRequestsCompleted = false;
        allRequestsCompleted = false;
    }

    public void request(Request r) {
        setActiveRequests(new RequestList(Arrays.asList(r), false));
        setQueuedRequests(new RequestList());
    }

    public void request(Request active, Request queue) {
        setActiveRequests(new RequestList(Arrays.asList(active), false));
        setQueuedRequests(new RequestList(Arrays.asList(queue), false));
    }

    public void request(RequestList requestList) {
        setActiveRequests(requestList);
        setQueuedRequests(new RequestList());
    }

    public void request(RequestList activeList, RequestList queuedList) {
        setActiveRequests(activeList);
        setQueuedRequests(queuedList);
    }

    public void addActiveRequest(Request request) {
        activeRequests.add(request);
        newRequests = true;
        activeRequestsCompleted = false;
        allRequestsCompleted = false;
    }

    public void addFormostActiveRequest(Request request) {
        activeRequests.addToForeFront(request);
        newRequests = true;
        activeRequestsCompleted = false;
        allRequestsCompleted = false;
    }

    public void queue(Request request) {
        queuedRequests.add(new RequestList(Arrays.asList(request), false));
    }

    public void queue(RequestList list) {
        queuedRequests.add(list);
    }

    public void replaceQueue(Request request) {
        setQueuedRequests(new RequestList(Arrays.asList(request), false));
    }

    public void replaceQueue(RequestList list) {
        setQueuedRequests(list);
    }

    public void replaceQueue(List<RequestList> lists) {
        setQueuedRequests(lists);
    }

     

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {

            @Override
            public void onStart(double timestamp) {
                stop();
                clearRequests();
            }
    
            @Override
            public void onLoop(double timestamp) {
                synchronized (Superstructure.this) {
                    if (!activeRequestsCompleted) {
                        if (newRequests) {
                            if (activeRequests.isParallel()) {
                                boolean allActivated = true;
                                for (Iterator<Request> iterator = activeRequests.getRequests().iterator(); iterator
                                        .hasNext();) {
                                    Request request = iterator.next();
                                    boolean allowed = request.allowed();
                                    allActivated &= allowed;
                                    if (allowed)
                                        request.act();
                                }
                                newRequests = !allActivated;
                            } else {
                                if (activeRequests.isEmpty()) {
                                    activeRequestsCompleted = true;
                                    return;
                                }
                                currentRequest = activeRequests.remove();
                                currentRequest.act();
                                newRequests = false;
                            }
                        }
                        if (activeRequests.isParallel()) {
                            boolean done = true;
                            for (Request request : activeRequests.getRequests()) {
                                done &= request.isFinished();
                            }
                            activeRequestsCompleted = done;
                        } else if (currentRequest.isFinished()) {
                            if (activeRequests.isEmpty()) {
                                activeRequestsCompleted = true;
                            } else if (activeRequests.getRequests().get(0).allowed()) {
                                newRequests = true;
                                activeRequestsCompleted = false;
                            }
                        }
                    } else {
                        if (!queuedRequests.isEmpty()) {
                            setActiveRequests(queuedRequests.remove(0));
                        } else {
                            allRequestsCompleted = true;
                        }
                    }
    
                }
            }
    
            @Override
            public void onStop(double timestamp) {
                
            }
    
        });
    }

    public void clearRequests() {
        activeRequests = new RequestList();
        queuedRequests.clear();
    }

    public RequestList disabledRequest() {
        return new RequestList(Arrays.asList(mDrive.openLoopRequest(DriveSignal.NEUTRAL)), true);
    }


    /*public void autoPositionControl() {
        RequestList state = new RequestList(Arrays.asList(mDrive.turnRequest(180, 2), 
            mIntake.stateRequest(IntakeControlState.IDLE_WHILE_DEPLOYED), mHopper.stateRequest(HopperControlState.OFF),
            mSpinner.openLopoRequest(0.0), mClimb.stateRequest(ClimbControlState.OFF)), true);
        RequestList queue = new RequestList(Arrays.asList(driveUntilControlPanelRequest(), mDrive.openLoopRequest(new DriveSignal(0, 0)),
            mSpinner.positionControlRequest()), false);
        request(state);
        replaceQueue(queue);
    }


    public void autoRotationControl() {
        RequestList state = new RequestList(Arrays.asList(mDrive.turnRequest(180, 2), 
            mIntake.stateRequest(IntakeControlState.IDLE_WHILE_DEPLOYED), mHopper.stateRequest(HopperControlState.OFF),
            mSpinner.openLopoRequest(0.0), mClimb.stateRequest(ClimbControlState.OFF)), true);
        RequestList queue = new RequestList(Arrays.asList(driveUntilControlPanelRequest(), mDrive.openLoopRequest(new DriveSignal(0, 0)),
            mSpinner.rotationControlRequest()), false);
        request(state);
        replaceQueue(queue);
    }*/

    public void autoShootSequence() {
        RequestList state = new RequestList(Arrays.asList(mDrive.alignToTargetRequest(), 
            mShooter.setVelocityFromVisionRequest(),
            mIntake.stateRequest(IntakeControlState.SCORING), mHopper.stateRequest(HopperControlState.OFF)), true);
        RequestList queue = new RequestList(Arrays.asList(mHopper.stateRequest(HopperControlState.INDEX)), false);
        request(state);
        replaceQueue(queue);
    }

    public void autoShootBalls(int balls) {
        RequestList state = new RequestList(Arrays.asList( 
            mDrive.alignToTargetRequest(), mShooter.setVelocityFromVisionRequest(), mIntake.stateRequest(IntakeControlState.SCORING), mHopper.stateRequest(HopperControlState.OFF)), true);
        RequestList queue = new RequestList(Arrays.asList(mDrive.openLoopRequest(new DriveSignal(0, 0)), mHopper.indexBallNumberRequest(balls)), false);
        request(state);
        replaceQueue(queue);
    }


    public void autoShootBalls(int balls, int rpm, double waitTime) {
        RequestList state = new RequestList(Arrays.asList( 
            mDrive.alignToTargetRequest(), mShooter.setVelocityAndWaitRequest(rpm), mIntake.stateRequest(IntakeControlState.SCORING), mHopper.stateRequest(HopperControlState.OFF)), true);
        RequestList queue = new RequestList(Arrays.asList(waitRequest(waitTime), mHopper.indexBallNumberRequest(balls)), false);
        request(state);
        replaceQueue(queue);
    }

    public void autoShootNoAlign(int balls, int rpm, double waitTime) {
        RequestList state = new RequestList(Arrays.asList(mShooter.setVelocityAndWaitRequest(rpm),
             mIntake.stateRequest(IntakeControlState.SCORING), mHopper.stateRequest(HopperControlState.OFF)), true);
        RequestList queue = new RequestList(Arrays.asList(waitRequest(waitTime), mHopper.indexBallNumberRequest(balls)), false);
        request(state);
        replaceQueue(queue);
    }


    public Request waitRequest(double seconds) {
        return new Request() {
            double startTime = 0;
            double waitTime = 1;

            @Override
            public void act() {
                startTime = Timer.getFPGATimestamp();
                waitTime = seconds;
            }

            @Override
            public boolean isFinished() {
                return (Timer.getFPGATimestamp() - startTime) >= waitTime;
            }
        };
    }





    public Request turnUntilSeesTargetRequest(boolean left) {
        return new Request(){
        
            @Override
            public void act() {
                mDrive.setOpenLoop(left ? new DriveSignal(-0.3, 0.3) 
                    : new DriveSignal(0.3, -0.3));
            }

            @Override
            public boolean isFinished() {
                return mLimelight.seesTarget();
            }
        };
    }


    

    /*public Request driveUntilControlPanelRequest() {
        Request request = new Request(){
        
            @Override
            public void act() {
                mDrive.setOpenLoop(new DriveSignal(0.3, 0.3));
            }

            @Override
            public boolean isFinished() {
                return mSpinner.seesControlPanel();
            }
        };

        request.withPrerequisite(mSpinner.deployedPrerequisite);

        return request;
    }*/





    @Override
    public void stop() {
        setActiveRequests(disabledRequest());
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {

    }

	public boolean isAtDesiredState() {
		return true;
	}


}

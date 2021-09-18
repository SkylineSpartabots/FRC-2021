/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.requests;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Add your docs here.
 */
public class RequestList {

    ArrayList<Request> requests;
    boolean parallel = false;

    public RequestList(List<Request> requests, boolean parallel) {
        this.requests = new ArrayList<>(requests.size());
        for(Request request : requests) {
            this.requests.add(request);
        }
        this.parallel = parallel;
    } 

    public RequestList() {
        this(Arrays.asList(new EmptyRequest()), false);
    }

    public RequestList(Request request) {
        this(Arrays.asList(request), false);
    }

    public static RequestList emptyList() {
        return new RequestList(new ArrayList<>(0), false);
    }

    public boolean isParallel() {
        return parallel;
    }

    public List<Request> getRequests() {
        return requests;
    }

    public void add(Request request) {
        requests.add(request);
    }

    public void addToForeFront(Request request) {
        requests.add(0, request);
    }

    public Request remove(){
        return requests.remove(0);
    }

    public boolean isEmpty() {
        return requests.isEmpty();
    }


}

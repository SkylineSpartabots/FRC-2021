/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.requests;

import java.util.ArrayList;
import java.util.List;

/**
 * Add your docs here.
 */
public abstract class Request {
    
    public abstract void act();

    public boolean isFinished() {
        return true;
    }

    public List<Prerequisite> prerequisites = new ArrayList<>();

    public void withPrerequisites(List<Prerequisite> reqs) {
        for(Prerequisite req : reqs) {
            prerequisites.add(req);
        }
    }

    public void withPrerequisite(Prerequisite req) {
        prerequisites.add(req);
    }

    public boolean allowed() {
        boolean reqsMet = true;
        for(Prerequisite req : prerequisites) {
            reqsMet &= req.met();
        }
        return reqsMet;
    }
}

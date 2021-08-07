/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.paths;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants;


public class PathGenerator {

    private static PathGenerator mInstance = null;
    public static PathGenerator getInstance() {
        if(mInstance == null) {
            mInstance = new PathGenerator();
        }
        return mInstance;
    }

    //constraints
    private final DifferentialDriveVoltageConstraint mAutoVoltageConstraint;
    private final DifferentialDriveKinematics mDriveKinematics;
    private final TrajectoryConfig mTrajectoryConfig;

    //trajectories
    private PathSet mPathSet = null;


    private PathGenerator() {
        mDriveKinematics = new DifferentialDriveKinematics(Constants.kDriveWheelTrackWidthMeters);
        mAutoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.kDriveKsVolts, Constants.kDriveKvVolts, Constants.kDriveKaVolts), 
            mDriveKinematics, 10);
        
        mTrajectoryConfig = new TrajectoryConfig(Constants.kDriveMaxVelocity, Constants.kDriveMaxAcceleration)
            .setKinematics(mDriveKinematics).addConstraint(mAutoVoltageConstraint);
        
    }

    public PathSet getPathSet() {
        return mPathSet;
    }

    public void generatePaths() {
        if(mPathSet == null) {
            double startTime = Timer.getFPGATimestamp();
            System.out.println("Generating trajectories...");
            mPathSet = new PathSet();
            System.out.println("Finished trajectory generation in: " + (Timer.getFPGATimestamp() - startTime) + " seconds");
        }
    }

    private Trajectory generatePath(Pose2d startPose, List<Translation2d> interiorWaypoints, Pose2d endPose, boolean isReverse) {
        mTrajectoryConfig.setReversed(isReverse);
        return TrajectoryGenerator.generateTrajectory(startPose, interiorWaypoints, endPose, mTrajectoryConfig);
    }


    public static void setStartPose(Pose2d startPose) {
        mAutoStartingPose = startPose;
    } 
    

    private static Pose2d mAutoStartingPose = new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0));

    private static Pose2d mTrenchEndPose = new Pose2d(new Translation2d(1, 0.0), new Rotation2d(0.0));

    private static Pose2d mTrenchRunShootPose = new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0));

    


    public class PathSet {

        public final Trajectory startPositionOneToTrenchEnd;
        //public final Trajectory startPositionTwoToTrenchEnd;
        //public final Trajectory startPositionThreeToOpponentBalls;
        //public final Trajectory opponentBallsToShootLocation;
        public final Trajectory trenchEndToShootLocation;


        private PathSet() {
            startPositionOneToTrenchEnd = getStartPositionOneToTrenchEnd();
            //startPositionTwoToTrenchEnd = getStartPositionTwoToTrenchEnd();

            //startPositionThreeToOpponentBalls = getStartPositionThreeToOpponentBalls();
            //opponentBallsToShootLocation = getOpponentBallsToShootLocation();

            trenchEndToShootLocation = getTrenchEndToShootLocation();
        }

    }



    private Trajectory getStartPositionOneToTrenchEnd() {
        List<Translation2d> innerWaypoints = new ArrayList<>();
        //innerWaypoints.add(new Translation2d(1, 0.5));
        innerWaypoints.add(new Translation2d(0.5, -0.1));

        return generatePath(mAutoStartingPose, innerWaypoints, mTrenchEndPose, false);
    }


    private Trajectory getStartPositionTwoToTrenchEnd() {
        List<Translation2d> innerWaypoints = new ArrayList<>();
        innerWaypoints.add(new Translation2d(1.0, 0));

        return generatePath(mAutoStartingPose, innerWaypoints, mTrenchEndPose, false);
    }

    private Trajectory getTrenchEndToShootLocation() {
        List<Translation2d> innerWaypoints = new ArrayList<>();
        //innerWaypoints.add(new Translation2d(0., 0.0));

        return generatePath(mTrenchEndPose, innerWaypoints, mAutoStartingPose, true);
    }

    private Trajectory getStartPositionThreeToOpponentBalls() {
        List<Translation2d> innerWaypoints = new ArrayList<>();
        innerWaypoints.add(new Translation2d(0.0, 0.0));

        return generatePath(mAutoStartingPose, innerWaypoints, mTrenchEndPose, true);
    }

    private Trajectory getOpponentBallsToShootLocation() {
        List<Translation2d> innerWaypoints = new ArrayList<>();
        innerWaypoints.add(new Translation2d(0.0, 0.0));

        return generatePath(mAutoStartingPose, innerWaypoints, mTrenchEndPose, true);
    }

    

    


}

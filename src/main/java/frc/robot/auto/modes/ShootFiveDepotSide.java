/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.modes;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.lib.util.DriveSignal;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.PerfectlyStraightDriveAction;
import frc.robot.auto.actions.WaitForRequestsAction;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Hopper.HopperControlState;
import frc.robot.subsystems.Intake.IntakeControlState;

/**
 * Add your docs here.
 */
public class ShootFiveDepotSide extends AutoModeBase {


    private Shooter mShooter = Shooter.getInstance();
    private Drive mDrive = Drive.getInstance();
    private Intake mIntake = Intake.getInstance();
    private Hopper mHopper = Hopper.getInstance();
    private Superstructure mSuperstructure = Superstructure.getInstance();

    
    @Override
    protected void routine() throws AutoModeEndedException {
        mShooter.setOpenLoop(0.2);
        mDrive.setOpenLoop(new DriveSignal(0, 0));

        mIntake.conformToState(IntakeControlState.INTAKE);
        mHopper.conformToState(HopperControlState.SENSORED_INTAKE);

        //motion for driving backwards and retrieving balls
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0.0), 1.8, 0.85));
        mShooter.shootAtSetRpm(4700);

        //motion for driving from steal to shooting pose
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0.0), 0.8, -0.85));
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(90.0), 3.5, -0.85));
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(15.0), 0.4, -0.6));
        mSuperstructure.autoShootNoAlign(5, 4700, 0.0);
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(15.0), 0.1, -0.6));
        mDrive.setOpenLoop(new DriveSignal(0.0, 0.0));

        runAction(new WaitForRequestsAction());

        mHopper.conformToState(HopperControlState.SENSORED_INTAKE);
        mIntake.conformToState(IntakeControlState.INTAKE);
        mShooter.shootAtSetRpm(4700);

        //motion for driving from shooting pose to ball pick up
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(15.0), 0.6, -0.6));
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(-15.0), 0.5, -0.5));

        //motion for pick up back to shooting position
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(-15.0), 0.5, 0.6));
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(15.0), 0.4, 0.6));
        mSuperstructure.autoShootNoAlign(5, 4700, 0.0);
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(15.0), 0.1, 0.6));
        mDrive.setOpenLoop(new DriveSignal(0.0, 0.0));


        runAction(new WaitForRequestsAction());

        mHopper.conformToState(HopperControlState.OFF);
        mIntake.conformToState(IntakeControlState.IDLE_WHILE_DEPLOYED);
        mShooter.setOpenLoop(0.0);






        
    }
}

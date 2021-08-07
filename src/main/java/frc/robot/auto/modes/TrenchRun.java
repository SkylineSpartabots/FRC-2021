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
import frc.robot.auto.actions.WaitAction;
import frc.robot.auto.actions.WaitForRequestsAction;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Hopper.HopperControlState;
import frc.robot.subsystems.Intake.IntakeControlState;


public class TrenchRun extends AutoModeBase {

    private Shooter mShooter = Shooter.getInstance();
    private Intake mIntake = Intake.getInstance();
    private Hopper mHopper = Hopper.getInstance();
    private Drive mDrive = Drive.getInstance();
    private Superstructure mSuperstructure = Superstructure.getInstance();


    @Override
    protected void routine() throws AutoModeEndedException {
        mShooter.setOpenLoop(0.0);
        mDrive.setOpenLoop(new DriveSignal(0, 0));

        mSuperstructure.autoShootBalls(3, 4850, 0);
        runAction(new WaitForRequestsAction());
        runAction(new WaitAction(0.2));

        mHopper.conformToState(HopperControlState.SENSORED_INTAKE);
        mShooter.setOpenLoop(0.1);
        mIntake.conformToState(IntakeControlState.INTAKE);

        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(-64.0), 1.12, 0.4));
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0.0), 4.05, 0.45));
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0.0), 0.85, 0.4));

        mShooter.shootAtSetRpm(4750);

        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0.0), 0.85, -0.6));
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(0.0), 2.3, -0.65));
        runAction(new PerfectlyStraightDriveAction(Rotation2d.fromDegrees(-14.0), 0.6, -0.55));
        mDrive.setOpenLoop(new DriveSignal(0.0, 0.0));
        
    
        mSuperstructure.autoShootBalls(5, 4750, 0.3);
        runAction(new WaitForRequestsAction());

        mHopper.conformToState(HopperControlState.OFF);
        mIntake.conformToState(IntakeControlState.IDLE_WHILE_DEPLOYED);
        mShooter.setOpenLoop(0.0);

    }
}

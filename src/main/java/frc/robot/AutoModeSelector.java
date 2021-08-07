/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.modes.AutoModeBase;
import frc.robot.auto.modes.CrossLine;
import frc.robot.auto.modes.DoNothing;
import frc.robot.auto.modes.ShootFiveDepotSide;
import frc.robot.auto.modes.ShootThree;
import frc.robot.auto.modes.TenBall;
import frc.robot.auto.modes.TrenchRun;

/**
 * Add your docs here.
 */
public class AutoModeSelector {
    
    public enum StartingPosition {
        TARGET_SIDE, MIDDLE, DEPOT_SIDE;
    }

    private enum DesiredMode {
        DO_NOTHING,
        CROSS_LINE,
        SHOOT_THREE_CROSS_LINE,
        SHOOT_THREE,
        TRENCH_RUN,
        STEAL_FIVE,
        SHOOT_TEN,
    }

    private DesiredMode mCachedDesiredMode = null;
    private StartingPosition mCachedStartingPosition = null;

    private SendableChooser<DesiredMode> mModeChooser;
    private SendableChooser<StartingPosition> mStartPositionChooser;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    public AutoModeSelector() {
        mStartPositionChooser = new SendableChooser<>();
        mStartPositionChooser.setDefaultOption("Target Side", StartingPosition.TARGET_SIDE);
        mStartPositionChooser.addOption("Middle", StartingPosition.MIDDLE);
        mStartPositionChooser.addOption("Depot Side", StartingPosition.DEPOT_SIDE);

        SmartDashboard.putData("Starting Position", mStartPositionChooser);

        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Trench Run", DesiredMode.TRENCH_RUN);
        mModeChooser.addOption("10 Ball", DesiredMode.SHOOT_TEN);
        mModeChooser.addOption("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.addOption("Cross Line", DesiredMode.CROSS_LINE);
        mModeChooser.addOption("Shoot Three and Cross Line", DesiredMode.SHOOT_THREE_CROSS_LINE);
        mModeChooser.addOption("Shoot Three Only", DesiredMode.SHOOT_THREE);
        mModeChooser.addOption("Steal Five", DesiredMode.STEAL_FIVE);
        SmartDashboard.putData("Auto mode", mModeChooser);
    }

    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();
        StartingPosition startingPosition = mStartPositionChooser.getSelected();
        if (mCachedDesiredMode != desiredMode || startingPosition != mCachedStartingPosition) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name()
                    + ", starting position->" + startingPosition.name());
            mAutoMode = getAutoModeForParams(desiredMode, startingPosition);
        }
        mCachedDesiredMode = desiredMode;
        mCachedStartingPosition = startingPosition;
    }

    private Optional<AutoModeBase> getAutoModeForParams(DesiredMode mode, StartingPosition position) {
        switch (mode) {
            case DO_NOTHING:
                return Optional.of(new DoNothing());
            case CROSS_LINE:
                return Optional.of(new CrossLine());
            case TRENCH_RUN:
                return Optional.of(new TrenchRun());
            case SHOOT_TEN:
                return Optional.of(new TenBall());
            case STEAL_FIVE:
                return Optional.of(new ShootFiveDepotSide());
            case SHOOT_THREE_CROSS_LINE:
                return Optional.of(new ShootThree(position, true));
            case SHOOT_THREE:
                return Optional.of(new ShootThree(position, false));
            default:
                break;
        }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.of(new DoNothing());
    }

    public void reset() {
        mAutoMode = Optional.empty();
        mCachedDesiredMode = null;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
        SmartDashboard.putString("StartingPositionSelected", mCachedStartingPosition.name());
    }

    public Optional<AutoModeBase> getAutoMode() {
        return mAutoMode;
    }
    
}

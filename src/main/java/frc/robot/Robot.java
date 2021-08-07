/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.controllers.OverridesController;
import frc.lib.controllers.Xbox;
import frc.lib.util.CrashTracker;
import frc.lib.util.DriveSignal;
import frc.lib.util.TelemetryUtil;
import frc.lib.util.Util;
import frc.lib.util.TelemetryUtil.PrintStyle;
import frc.robot.auto.AutoModeExecutor;
import frc.robot.auto.modes.AutoModeBase;
import frc.robot.auto.modes.CrossLine;
import frc.robot.loops.Looper;
import frc.robot.paths.PathGenerator;
import frc.robot.subsystems.AirCompressor;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spinner;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Climb.ClimbControlState;
import frc.robot.subsystems.Hopper.HopperControlState;
import frc.robot.subsystems.Intake.IntakeControlState;
import frc.robot.subsystems.Limelight.LedMode;


public class Robot extends TimedRobot {

  //Loopers
  private final Looper mEnabledLooper = new Looper();
  private final Looper mDisabledLooper = new Looper();

  //Drive team Controllers
  private final Xbox mDriveController = new Xbox(0);
  private final Xbox mOperatorController = new Xbox(1);
  private final OverridesController mOverridesController = OverridesController.getInstance();


  //Subsystems
  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

  private final Intake mIntake = Intake.getInstance();
  private final Hopper mHopper = Hopper.getInstance();
  //private final Spinner mSpinner = Spinner.getInstance();
  private final Climb mClimb = Climb.getInstance();
  //private final LED mLED = LED.getInstance();
  private final Shooter mShooter = Shooter.getInstance();
  private final Drive mDrive = Drive.getInstance();
  private final AirCompressor mAirCompressor = AirCompressor.getInstance();
  private final Superstructure mSuperstructure = Superstructure.getInstance();

  private Limelight mLimelight = Limelight.getInstance();
  
  private double mMatchStartTimestamp = Double.POSITIVE_INFINITY;


  private final PathGenerator mPathGenerator = PathGenerator.getInstance();
  
  private AutoModeSelector mAutoModeSelector = new AutoModeSelector();
  private AutoModeExecutor mAutoModeExecutor;



  Robot() {
    CrashTracker.logRobotConstruction();
  }


  @Override
  public void robotInit() {
    try {
      CrashTracker.logRobotInit();
    
      mSubsystemManager.setSubsystems(
        mDrive,
        mLimelight,
        mIntake,
        mHopper,
        mShooter,
        mClimb,
        mAirCompressor,
        mSuperstructure
      );

      mSubsystemManager.registerEnabledLoops(mEnabledLooper);
      mSubsystemManager.registerDisabledLoops(mDisabledLooper);

      mSubsystemManager.zeroSensors();

      
      mPathGenerator.generatePaths();
      mAutoModeSelector.updateModeCreator();
      
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }


  @Override
  public void robotPeriodic() {
    try {
      mSubsystemManager.outputToSmartDashboard();
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
    
  }

  @Override
  public void disabledInit() {
    try {
      CrashTracker.logDisabledInit();
      mEnabledLooper.stop();
      

      if(mAutoModeExecutor != null) {
        mAutoModeExecutor.stop();
      }

      mAutoModeSelector.reset();
      mAutoModeSelector.updateModeCreator();
      mAutoModeExecutor = new AutoModeExecutor();
    

      mDisabledLooper.start();

      mLimelight.setLed(LedMode.OFF);
      mDrive.setBrakeMode(false);

    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  @Override
  public void disabledPeriodic() {
    try {

      mAutoModeSelector.updateModeCreator();

      Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
      if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
        System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
        mAutoModeExecutor.setAutoMode(autoMode.get());
      }

  

    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  @Override
  public void autonomousInit() {
    try {
      CrashTracker.logAutoInit();
      mDisabledLooper.stop();

      //Zero sensors and robot state accordingly
      mDrive.zeroSensors();
      mSubsystemManager.zeroSensors();
      mDrive.resetOdometry(new Pose2d(new Translation2d(0.0, 0.0), mDrive.getHeading()));
      
      mEnabledLooper.start();

      mAutoModeExecutor.start();

      mLimelight.setVisionMode();

    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
    
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    try {

    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  @Override
  public void teleopInit() {
    try {
      mDisabledLooper.stop();

      if(mAutoModeExecutor != null) {
        mAutoModeExecutor.stop();
      }

      mMatchStartTimestamp = Timer.getFPGATimestamp();

      mLimelight.setDriveMode();
      
      shooterClimbShutoff = false;
      deployIntake = true;
      automatedControlEnabled = false;

      mEnabledLooper.start();
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
    }
  }

  /**
   * This function is called periodically during operator control.
   */

  private boolean deployIntake;
  private boolean automatedControlEnabled;
  private boolean shooterClimbShutoff;


  @Override
  public void teleopPeriodic() {
    try {
      mDriveController.update();
      mOperatorController.update();
      mOverridesController.update();


      
      //driverControl();

      SmartDashboard.putNumber("Shooter Calibration Target RPM", 5700);
      shooterCalibration();
    
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
    
  }

  @Override
  public void testInit() {
    try {
      CrashTracker.logTestInit();
      mDisabledLooper.stop();
      
      
     // mAutoModeExecutor.stop();
      //mTestModeExecutor.start();

      mEnabledLooper.start();
      
      
    } catch(Throwable t) {
      CrashTracker.logThrowableCrash(t);
    }
  }

  @Override
  public void testPeriodic() { 
    try {

      
    } catch(Throwable t) {
      CrashTracker.logThrowableCrash(t);
    }
  }

  



  public void driverControl() {
    SmartDashboard.putBoolean("Automated Control", automatedControlEnabled);
    SmartDashboard.putBoolean("Shooter Is Idling?", !shooterClimbShutoff);


    /* Complete automation controls:
        Start auto shoot sequence - B
        Auto Align to target - right bumper
    */
   if(mOperatorController.bButton.isBeingPressed()) {
      if(!automatedControlEnabled) {
        mLimelight.setVisionMode();
        mSuperstructure.autoShootSequence();
        mDriveController.rumble(0.8);
        if(mLimelight.isCloseDistance() && mLimelight.seesTarget()) {
          mOperatorController.rumble(0.6);
        } else {
          mOperatorController.rumble(0.0);
        }
        automatedControlEnabled = true;
      }
    } else {
      if(automatedControlEnabled) {
        mSuperstructure.clearRequests();
        mOperatorController.rumble(0.0);
        mOperatorController.rumble(0.0);
      }
      automatedControlEnabled = false;
    }


    /* Drive Controls:
        Arcade Drive - Left joy y, right joy x (DC)
        Auto-align to targt - Right bumper (OC)
    */
    if(!automatedControlEnabled) {
      if(mOperatorController.rightBumper.isBeingPressed()) {
        mLimelight.setVisionMode();
        mDrive.setAlignToTarget();
        mDriveController.rumble(0.2);
      } else {
        mDrive.setArcadeDrive(mDriveController.getY(Hand.kLeft), mDriveController.getX(Hand.kRight));
        mLimelight.setDriveMode();
        mDriveController.rumble(0.0);
      }
    }


   /* Intake Controls
        Intake On with Auto Index - Left Bumper
        Intake On - A 
        Retract Intake - Y
  
    */
    if(!automatedControlEnabled) {
      if(mOperatorController.yButton.wasActivated()) {
        deployIntake = false;
      }
  
      if(mOperatorController.leftBumper.isBeingPressed() ) {
        mIntake.conformToState(IntakeControlState.INTAKE);
        deployIntake = true;

      } else {
        if(deployIntake) {
          if(mOperatorController.xButton.isBeingPressed()) {
            mIntake.conformToState(IntakeControlState.OUTAKE);
          } else {
            mIntake.conformToState(IntakeControlState.STORE);
          }
        } else {
          mIntake.conformToState(IntakeControlState.OFF);
        }
      }
    }

   
    /* Shooter controls:
        Start shooter ramp - Left paddle
        Start and continue auto shooter sequence - B
        Open loop shooter control - Right trigger
    */
    if(!automatedControlEnabled) {
      if(mOperatorController.backButton.isBeingPressed()) {
        mShooter.shootAtSetRpm(4700);
      } else if(mOperatorController.getTriggerAxis(Hand.kLeft) > 0.1) {
        mShooter.setOpenLoop(Util.limit(mOperatorController.getTriggerAxis(Hand.kLeft)-0.35, 0.2, 0.5));
      } else {
        if(!shooterClimbShutoff) {
          ///mShooter.setOpenLoop(0.35); //TODO: Change back
          mShooter.setOpenLoop(0.0);
        } else {
          mShooter.setOpenLoop(0.0);
        }
      }
    }

    if(mDriveController.aButton.isBeingPressed()) {
      mShooter.setControllerConstants();
    }
    

    /* Hopper Control:
        sensored index - right trigger
    */
    if(!automatedControlEnabled) {
      if(mOperatorController.rightTrigger.isBeingPressed()) {
        mHopper.conformToState(HopperControlState.INDEX);
      } else if(mOperatorController.leftBumper.isBeingPressed() || mOperatorController.aButton.isBeingPressed()) {
        mHopper.setSlowIndexState(mOperatorController.aButton.isBeingPressed());
        mHopper.conformToState(HopperControlState.SENSORED_INTAKE);
      } else if(mOperatorController.getY(Hand.kLeft) < -0.2) {
        mHopper.conformToState(HopperControlState.REVERSE);
      } else {
        mHopper.conformToState(HopperControlState.OFF);
      }
    }



    /* Climb Control:
        Deploy climb: dpad up
        Retract climb: dpad down
        Winch up: dpad right
    */

    

    if(Timer.getFPGATimestamp() - mMatchStartTimestamp > 95) {
      if(mOperatorController.dpadUp.isBeingPressed()) {
        shooterClimbShutoff = true;
        mClimb.conformToState(ClimbControlState.RAISE_HOOK);
      } else if(mOperatorController.dpadDown.isBeingPressed()) {
        mClimb.conformToState(ClimbControlState.LOWER_HOOK);
      } else if(mOperatorController.dpadRight.isBeingPressed()) {
        mClimb.conformToState(ClimbControlState.WINCH_UP);
      } else {
        if(mOperatorController.getY(Hand.kRight) > 0.2) {
          mClimb.conformToState(ClimbControlState.OVERRIDE_RAISE_HOOK);
        } else if(mOperatorController.getY(Hand.kRight) < -0.2) {
          mClimb.conformToState(ClimbControlState.OVERRIDE_LOWER_HOOK);
        } else {
          mClimb.conformToState(ClimbControlState.OFF);
        }
      }
    } else {
      if(mOperatorController.dpadDown.isBeingPressed()) {
        mClimb.conformToState(ClimbControlState.LOWER_HOOK);
      } else {
        mClimb.conformToState(ClimbControlState.OFF);
      }
    }
    

    


    if(mOperatorController.startButton.isBeingPressed()) {
      shooterClimbShutoff = false;
    }


    
    /* Spinner controls:
        Start rotation control - Dpad up
        Start position control - Dpad down
        Retract spinner - Left d pad
        Open loop on spinner - right joy x
    */
    
    
  
  }


  private double mShootCalibrationVelocity = 0.0;
  public void shooterCalibration() {
    mLimelight.setVisionMode();
    mShootCalibrationVelocity = SmartDashboard.getNumber("Shooter Calibration Target RPM", mShootCalibrationVelocity);
    SmartDashboard.putNumber("Limelight Distance", mLimelight.getDistance());
  
    if(mDriveController.xButton.wasActivated()) {
      TelemetryUtil.print("X: " + mLimelight.getDistance() + " Y: " + mShootCalibrationVelocity, PrintStyle.LOGGER_PRO, true);
    }

    if(mDriveController.aButton.isBeingPressed()) {
      mShooter.shootAtSetRpm(mShootCalibrationVelocity);
    } else {
      mShooter.setOpenLoop(0.0);
    }



    if(mDriveController.rightBumper.isBeingPressed()) {
      mDrive.setAlignToTarget();
    } else {
      mDrive.setArcadeDrive(mDriveController.getY(Hand.kLeft), mDriveController.getX(Hand.kRight));
    }


   

    if(mDriveController.leftTrigger.isBeingPressed()) {
      mHopper.conformToState(HopperControlState.SENSORED_INDEX);
    } else {
      mHopper.conformToState(HopperControlState.OFF);
    }

    /*if(mDriveController.leftBumper.isBeingPressed()) {
      mIntake.conformToState(IntakeControlState.INTAKE);
    } else {
      mIntake.conformToState(IntakeControlState.IDLE_WHILE_DEPLOYED);
    }*/
  }

 
}


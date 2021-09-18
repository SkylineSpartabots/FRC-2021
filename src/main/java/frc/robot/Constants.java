/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


public class Constants {

	// Timeout for CAN commands and error checking
	public static final int kTimeOutMs = 10;

	// Cycle speed for looper threads
	public static final double kLooperDt = 0.01;

	// Drive
	public static final double kDriveWheelTrackWidthMeters = 23.0 * 0.0254; //26.125
	public static final double kDriveWheelDiameter = 6.0;
	public static final double kDriveWheelDiameterInMeters = kDriveWheelDiameter * 0.0254;
	public static final double kDriveWheelRadiusInches = kDriveWheelDiameter / 2.0;
	public static final double kDriveWheelTrackRadiusMeters = kDriveWheelTrackWidthMeters / 2.0;
	
	public static final double kQuickStopThreshold = 0;
	public static final double kQuickStopAlpha = 0.05;

	public static final double kPerfectlyDriveProportion = 0.005;
	
	public static final double kDriveKaVolts = 0.08;
	public static final double kDriveKvVolts = 0.05;
	public static final double kDriveKsVolts = 0.08;
	public static final double kDriveMaxVelocity = 1; //4.75
	public static final double kDriveMaxAcceleration = 1; //3.166
	public static final double kDrivePathingProportion = 0.004;

	public static final double kDriveTurnKs = 0.035; //0.04

	// Shooter	
	public static final double kShooterRampRate = 0.2;
	public static final int kShooterKfBufferSize = 20;
	public static final double kShooterkP = 0.0; //0.01
	public static final double kShooterkI = 0.0000; //0.00005
	public static final double kShooterkD = 0;
	public static final double kShooterkF = 0.031;
	public static final double kShooterHoldkF = 0.048;
	public static final int kShooterIZone = 0;
	public static final double kShooterStartOnTargetRpm = 200;
	public static final double kShooterStopOnTargetRpm = 150;
	public static final int kShooterMinOnTargetSamples = 20;
	public static final double kRawVelocityToRpm = 1195.0/600.0;
	public static final int kFarShootDistanceThreshold = 180;

	// Limelight
	public static final double kLensHeight = 25.75;
	public static final double kLensHorizontalAngle = 15;
	public static final double kTargetHeight = 98.5;

	
	//Spinner
	public static final int kControlPanelProximityThreshold = 0;
	public static final int kNeoPPR = 42;
	public static final double kSpinnerWheelDiameterInches = 2.0;
	public static final int kControlPanelDiameterInches = 32;
	public static final double kSpinnerCountsPerInch = kNeoPPR / (Math.PI * kSpinnerWheelDiameterInches);
	public static final double kCountsPerControlPanelRotation = kControlPanelDiameterInches * kSpinnerCountsPerInch;
	
	//Climb
	public static final double kSlideDownToWinchTransitionTime = 0.1;
	public static final double kSlideDownEncoderTarget = 100;
	public static final double kHookSlideWaitHeightThreshold = 900;
	public static final double kClimbMaxHeight = 18800; //18200 sout is op
	
	// Air Compressor
	public static final double kCompressorShutOffCurrent = 60;

	public static final double kRotationControlPercentOutput = 0;

	public static final double kStandardShootVelocity = 4600;

	public static final double kDriveAlignControlGainSchedule = 3.5;

	public static final int kIndexSensorThreshold = 100;

	public static final double kStartUnjamTimeThreshold = 1;

	public static final double kStopUnjamTime = 0.4;

	public static final double kUnjamCurrentThreshold = 30.0;

	

	

	

}

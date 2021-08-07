/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.sensors;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;


public class ColorSensor {

    private static ColorSensor mInstance = null;
    public static ColorSensor getInstance() {
        if(mInstance == null) {
            mInstance = new ColorSensor();
        }
        return mInstance;
    }

    private final Color kBlueTarget = ColorMatch.makeColor(0.109375, 0.427978515625, 0.462646484375);
    private final Color kGreenTarget = ColorMatch.makeColor(0.149169921875 , 0.597412109375 , 0.253662109375);
    private final Color kRedTarget = ColorMatch.makeColor(0.5498046875, 0.330810546875, 0.119384765625);
    private final Color kYellowTarget = ColorMatch.makeColor(0.31787109375, 0.570556640625, 0.111572265625);
    private final Color kWhiteTarget = ColorMatch.makeColor(0.2861328125, 0.48291015625, 0.23095703125);

    private final ColorSensorV3 mColorSensor;
    private final ColorMatch mColorMatcher;

    public enum Colors {
        RED,
        GREEN,
        BLUE,
        YELLOW,
        UNKNOWN;
    }

    private  ColorSensor() {
        mColorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        
        mColorMatcher = new ColorMatch();
        mColorMatcher.addColorMatch(kBlueTarget);
        mColorMatcher.addColorMatch(kGreenTarget);
        mColorMatcher.addColorMatch(kRedTarget);
        mColorMatcher.addColorMatch(kYellowTarget);
        mColorMatcher.addColorMatch(kWhiteTarget);
        
    }

    public int getProximity() {
        return mColorSensor.getProximity();
    }

    public boolean hasReachedPanel() {
        return getProximity() < Constants.kControlPanelProximityThreshold;
    }

    public Colors getColor() {
        Color detectedColor = mColorSensor.getColor();
        ColorMatchResult match = mColorMatcher.matchClosestColor(detectedColor);
    
        if(match.color == kBlueTarget) {
            return Colors.BLUE;
        } else if (match.color == kRedTarget) {
            return Colors.RED;
        } else if (match.color == kGreenTarget) {
            return Colors.GREEN;
        } else if (match.color == kYellowTarget) {
            return Colors.YELLOW;
        } else {
            return Colors.UNKNOWN;
        }
    }

}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.states;

/**
 * Add your docs here.
 */
public class LEDState {

    public static final LEDState kOff = new LEDState(0.0, 0.0, 0.0);

    public static final LEDState kRed = new LEDState(1.0, 0.0, 0.0);
    public static final LEDState kGreen = new LEDState(0.0, 1.0, 0.0);
    public static final LEDState kBlue = new LEDState(0.0, 0.0, 1.0);

    public static final LEDState kWhite = new LEDState(0.33, 0.33, 0.33);
    public static final LEDState kPurple = new LEDState(0.5, 0.0, 0.5);
    public static final LEDState kYellow = new LEDState(0.5, 0.5, 0.0);
    public static final LEDState kOrange  = new LEDState(0.79, 0.21, 0.0);

    public static final LEDState kSpartaGreen = new LEDState(0.6352, 0.7058, 0.6588);

    public double red;
    public double green;
    public double blue;

    public LEDState(double r, double g, double b) {
        blue = b;
        red = r;
        green = g;
    }

    public void copyFrom(LEDState other) {
        this.red = other.red;
        this.green = other.green;
        this.blue = other.blue;
    }
}

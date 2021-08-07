/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.util;

/**
 * A Double that can be interpolated using the InterpolatingTreeMap.
 *
 * @see InterpolatingTreeMap
 */
public class InterpolatingDouble implements Interpolable<InterpolatingDouble>, InverseInterpolable<InterpolatingDouble>,
        Comparable<InterpolatingDouble> {
    public Double value = 0.0;

    public InterpolatingDouble(Double val) {
        value = val;
    }

    @Override
    public InterpolatingDouble interpolate(InterpolatingDouble other, double x) {
        Double dydx = other.value - value;
        Double searchY = dydx * x + value;
        return new InterpolatingDouble(searchY);
    }

    @Override
    public double inverseInterpolate(InterpolatingDouble upper, InterpolatingDouble query) {
        double upper_to_lower = upper.value - value;
        if (upper_to_lower <= 0) {
            return 0;
        }
        double query_to_lower = query.value - value;
        if (query_to_lower <= 0) {
            return 0;
        }
        return query_to_lower / upper_to_lower;
    }

    @Override
    public int compareTo(InterpolatingDouble other) {
        if (other.value < value) {
            return 1;
        } else if (other.value > value) {
            return -1;
        } else {
            return 0;
        }
    }

}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.util;

/**
 * Interpolable is an interface that interpolates between 2 values. Given two endpoints
 * and an interpolation parameter, it calculates a new interpolable representing the interpolated value
 */
public interface Interpolable<T> {

    /** 
     * @param other The value of the upper bound
     * @param x The requested value between 0 and 1
     * @return Interpolable <T> The estimated average between the surrounding data
     */
    T interpolate(T other, double x);
}

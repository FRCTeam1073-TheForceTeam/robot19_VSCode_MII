/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add methods here that can be used generally and universally
 */
public class Utilities {

    /**
     * Deadzone filter that uses the drivetrain deadzone
     * @param val value that you want filtered
     * @return scaled val, unless it is above or below the drivetrain deadzone value in Presets
     * @see Presets.java
     */
    public static double deadzone(double val) {
        return deadzone(val, Presets.driveDeadzone);
    }

    /**
     * Deadzone filter that filters out values inside the deadzone, and scales values outside
     * @param val value that you want filtered
     * @param deadzone value that acts as the minimum and maximum for the filter
     * @return scaled val, unless it is above or below the deadzone parameter
     */
    public static double deadzone(double val, double deadzone) {
        if (Math.abs(val) < deadzone) val = 0;
        else val = Math.signum(val) * Math.max(0, (Math.abs(val) - deadzone) / (1 - deadzone));
        return val;
    }
    
    /**
     * Clipping filter that filters values larger than max or smaller than min to the respective values
     * @param val value that you want clipped
     * @param min minimum value that the value can be
     * @param max maximum value that the value can be
     * @return value, unless it is outside of the range, in which case it is the respective max or min value
     */
    public static double clip(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    public static double clip(double val, double limit) {
        return clip(val, -limit, limit);
    }

    public static double clip(double val) {
        return clip(val, -1, 1);
    }
}

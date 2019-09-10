/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Utilities;
import frc.robot.XboxController;

/**
 * @author Ben
 * A class that handles drive controls for motors.
 * Not *explicitly* meant to be used in real code, but
 * more as a utility for short-term testing (eg expos)
 * when there isn't time to program an entire drive
 * control program.
 */
public class ControlledMotorAnalog extends ControlledMotor {
    private WPI_TalonSRX motor;
    private XboxController controller;
    private int axis;
    private double value;
    private double scale;
    private boolean deadzone;

    /**
     * @param motor_ The WPI_TalonSRX to be controlled.
     * @param axis_ The axis on the selected controller to be mapped to motor output.
     */
    public ControlledMotorAnalog(WPI_TalonSRX motor_, int axis_) {
        this(motor_, Robot.oi.driverControl, axis_, 1.0, true);
    }

    /**
     * @param motor_ The WPI_TalonSRX to be controlled.
     * @param axis_ The axis on the selected controller to be mapped to motor output.
     */
    public ControlledMotorAnalog(WPI_TalonSRX motor_, int axis_, double scale_) {
        this(motor_, Robot.oi.driverControl, axis_, scale_, true);
    }


    /**
     * @param motor_ The WPI_TalonSRX to be controlled.
     * @param controller_ The controller with the input axis.
     * @param axis_ The axis on the selected controller to be mapped to motor output.
     */
    public ControlledMotorAnalog(WPI_TalonSRX motor_, XboxController controller_, int axis_) {
        this(motor_, controller_, axis_, 1.0, true);
    }

    /**
     * @param motor_ The WPI_TalonSRX to be controlled.
     * @param controller_ The controller with the input axis.
     * @param axis_ The axis on the selected controller to be mapped to motor output.
     * @param scale_ The scale factor that converts inputs to outputs.
     * @param deadzone_ Whether deadzones are enabled.
     */
    public ControlledMotorAnalog(WPI_TalonSRX motor_, XboxController controller_, int axis_, double scale_,boolean deadzone_) {
        motor = motor_;
        axis = axis_;
        controller = controller_;
        scale = scale_;
        deadzone = deadzone_;
        value = 0;
    }

    public void init() {
        motor.set(0);
    }

    public void execute() {
        value = controller.getRawAxis(axis) * scale;
        double adjustedValue = Utilities.deadzone(value);
        motor.set(value);
    }

}

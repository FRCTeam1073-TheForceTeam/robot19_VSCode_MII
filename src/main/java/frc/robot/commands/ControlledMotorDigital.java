/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.buttons.JoystickButton;
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
 * This version uses individual buttons.
 */
public class ControlledMotorDigital extends ControlledMotor {
    private WPI_TalonSRX motor;
    private XboxController controller;
    private JoystickButton button;
    private double valueOff = 0;
    private double valueOn = 0.5;

    /**
     * @param motor_ The WPI_TalonSRX to be controlled.
     * @param button_ The axis on the selected controller to be mapped to motor output.
     */
    public ControlledMotorDigital(WPI_TalonSRX motor_, XboxController controller_, JoystickButton button_) {
        this(motor_, controller_, button_, 0.0, 1.0);
    }

    /**
     * @param motor_ The WPI_TalonSRX to be controlled.
     * @param button_ The axis on the selected controller to be mapped to motor output.
     * @param off The motor value when the button is not pressed.
     * @param on The motor value when the button is pressed.
     */
    public ControlledMotorDigital(WPI_TalonSRX motor_, XboxController controller_, JoystickButton button_, double off, double on) {
        motor = motor_;
        controller = controller_;
        button = button_;
        valueOff = off;
        valueOn = on;
    }

    public void init() {
        motor.set(0);
    }

    public void execute() {
        motor.set(button.get() ? valueOff : valueOn);
    }

}

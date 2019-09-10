/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.XboxController;

/**
 * A class to automate ControlledMotorAnalog objects on a higher level.
 * If you really don't need anything but low-quality code that maps input
 * axes to output axes, this is your best bet. It takes an array of
 * ControlledMotorAnalog objects and runs all of them the way it should.
 */
public class ControlledMotorGroup extends Command {
  private ArrayList<ControlledMotor> controllers;
  private JoystickButton abortSignal;
  private int controllerAmt = 0;

  public ControlledMotorGroup(ControlledMotor[] controllers_,Subsystem system) {
    this(controllers_, Robot.oi.driverCancel,system);
  }

  public ControlledMotorGroup(ControlledMotor[] controllers_, JoystickButton abortSignal_,Subsystem system) {
    requires(system);
    controllers = new ArrayList<ControlledMotor>();
    for (int i = 0; i < controllers_.length; i++) {
      controllers.add(controllers_[i]);
    }
    controllerAmt = controllers.size();
    abortSignal = abortSignal_;
  }
  
  public void addMotor(ControlledMotor cm) {
    controllers.add(cm);
    controllerAmt = controllers.size();
  }

  @Override
  protected void initialize() {
    for (int i = 0; i < controllerAmt; i++) {
      controllers.get(i).init();
    }
  }

  @Override
  protected void execute() {
    for (int i = 0; i < controllerAmt; i++) {
      controllers.get(i).execute();
    }
  }

  @Override
  protected boolean isFinished() {
    return abortSignal.get();
  }

}

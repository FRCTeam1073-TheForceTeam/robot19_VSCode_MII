/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.*;
import frc.robot.commands.*;
import frc.robot.RobotMap;

/**
 * Drivetrain Subsystem
 * Methods:
 * tankSet(double left, double right) - sets sides to speeds individually
 * deadzoneFilter(double left, double right) - checks if inputs are outside deadzone, then passes them to tankSet
 * tankDrive(double forward, double rotation) - mixes forward and rotational movement, then passes left and right values to deadzoneFilter
 */
public class Drivetrain extends Subsystem {

  public Drivetrain() {

  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveControls());
  }

}

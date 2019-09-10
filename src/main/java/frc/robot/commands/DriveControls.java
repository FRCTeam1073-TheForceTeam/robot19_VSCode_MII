/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.*;

public class DriveControls extends Command {

  double forward, rot;
  int loops=0;

  public DriveControls() {
    requires(Robot.drivetrain);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    forward = Robot.oi.getDrAxis(1);
    rot = Robot.oi.getDrAxis(5);
    System.out.println("[ALIVE "+loops+" ] " + forward + ", " + rot);
    Robot.drivetrain.setVelocity(forward * 6.5, rot * 6.5);
    Robot.oi.driverControl.rumble(Math.abs(forward), Math.abs(rot));
    loops++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

}

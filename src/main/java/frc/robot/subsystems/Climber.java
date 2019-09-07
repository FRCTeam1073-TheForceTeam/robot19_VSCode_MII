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

public class Climber extends Subsystem {
    private WPI_TalonSRX climberLeft = RobotMap.leftClimber;
    private WPI_TalonSRX climberRight = RobotMap.rightClimber;
    private WPI_TalonSRX climberWheels = RobotMap.climbWheels;
    public Hatch(){
    }
    @Override
    public void initDefaultCommand(){
        setDefaultCommand(new ClimberControls());
    }
    public void setClimber(double left, double right){
        climberLeft.set(ControlMode.PercentOutput, left);
        climberRight.set(ControlMode.PercentOutput, right);
    }
    public void setIntake(double power){
        intake.set(ControlMode.PercentOutput, power);
    }
}
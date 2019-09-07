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

public class Hatch extends Subsystem {
    private WPI_TalonSRX lift = RobotMap.hatchLift;
    private WPI_VictorSPX liftFollower = RobotMap.hatchLiftFollower;
    private WPI_TalonSRX intake = RobotMap.hatchCollect;
    public Hatch(){
        liftFollower.follow(lift);
    }
    @Override
    public void initDefaultCommand(){
        setDefaultCommand(new HatchControls());
    }
    public void setLift(double power){
        lift.set(ControlMode.PercentOutput, power);
    }
    public void setIntake(double power){
        intake.set(ControlMode.PercentOutput, power);
    }
}
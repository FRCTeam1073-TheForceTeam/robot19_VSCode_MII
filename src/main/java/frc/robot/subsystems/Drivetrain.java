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
  public final WPI_TalonSRX rightLeader = RobotMap.rightLeader;
	public final WPI_VictorSPX rightFollower = RobotMap.rightFollower;
	public final WPI_VictorSPX rightFollower2 = RobotMap.rightFollower2;
  public final WPI_TalonSRX leftLeader = RobotMap.leftLeader;
	public final WPI_VictorSPX leftFollower = RobotMap.leftFollower;
  public final WPI_VictorSPX leftFollower2 = RobotMap.leftFollower2;
  public double wheelRadius;
  public double drivebaseWidth;
  public final int encoderTickDensity=1440;//May not be correct, will require knowing what encoders will be used.
  public final double drivetrainGearRatio=1;//Placeholder for actual gear ratio on robot (not sure what it is)
  public double rotationRateConversionConstant;
  public Drivetrain(double wheelRadius_,double drivebaseWidth_) {
    wheelRadius=wheelRadius_;
    drivebaseWidth=drivebaseWidth_;

    //Constant to convert (wheel radians)/sec to (motor ticks)/100ms, the unit WPI_TalonSRX.set works with in velocity mode.
    //Used in setVelocity()
    rotationRateConversionConstant=0.1*(1/(2*Math.PI))*encoderTickDensity/drivetrainGearRatio;

    // rightLeader
    // rightFollower
    // rightFollower2
    // leftLeader
    // leftFollower
    // leftFollower2

    /** Motor config reset */
    rightLeader.configFactoryDefault();
    rightFollower.configFactoryDefault();
    rightFollower2.configFactoryDefault();
    leftLeader.configFactoryDefault();
    leftFollower.configFactoryDefault();
    leftFollower2.configFactoryDefault();

    /** Motor safety config */
    rightLeader.setSafetyEnabled(false);
    rightFollower.setSafetyEnabled(false);
    rightFollower2.setSafetyEnabled(false);
    leftLeader.setSafetyEnabled(false);
    leftFollower.setSafetyEnabled(false);
    leftFollower2.setSafetyEnabled(false);

    /** Motor neutral mode config */
    rightLeader.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);
    rightFollower2.setNeutralMode(NeutralMode.Brake);
    leftLeader.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Brake);
    leftFollower2.setNeutralMode(NeutralMode.Brake);

    /** Motor direction config */
    rightLeader.setInverted(true);
    rightFollower.setInverted(true);
    rightFollower2.setInverted(true);
    leftLeader.setInverted(false);
    leftFollower.setInverted(false);
    leftFollower2.setInverted(false);

    /** Motor following config */
    rightFollower.follow(rightLeader);
    rightFollower2.follow(rightLeader);
    leftFollower.follow(leftLeader);
    leftFollower2.follow(leftLeader);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveControls());
  }

  /** Sets the motors to the input value */
  public void tankSet(double left, double right) {
    leftLeader.set(ControlMode.PercentOutput, left);
    rightLeader.set(ControlMode.PercentOutput, right);
  }

  /** Checks if the input is outside of the deadzone*/
  //public void deadzoneFilter(double left, double right) {
    public static double deadzoneFilter(double input) {
      double output;
      if(input < Presets.deadzone && input > -Presets.deadzone) output = 0;
      else output =Math.signum(input)*Math.max(0,(Math.abs(input)-Presets.deadzone)/(1-Presets.deadzone));
      return output;
    //if(right < Presets.deadzone && right > -Presets.deadzone) right = 0;
    //else right=Math.signum(right)*Math.max(0,(Math.abs(right)-Presets.deadzone)/(1-Presets.deadzone));
    
  }

  /** This SHOULD MAYBE take both of the input values and produce movement that does not favor either input, but is far from final */
  public void tankDrive(double forward, double rotation) {
    double mid, right, left;
    mid = forward - (Math.abs(rotation) * Math.signum(forward));
    if(rotation > 0) {
      right = mid - (Math.abs(rotation) * Math.signum(forward));
      left = mid + (Math.abs(rotation) * Math.signum(forward));
    }
    else {
      right = mid + (Math.abs(rotation) * Math.signum(forward));
      left = mid - (Math.abs(rotation) * Math.signum(forward));
    }
    

    tankSet(deadzoneFilter(left), deadzoneFilter(right));
  }
  /**Tank drive based on real speed and rotation rate (translate in cm/s, rotate in radians/s) 
   * WIP, obviously.
  */
  public void velocityDefinedDrive(double translate, double rotate){
    /*
    Transformation is
    [1/r  L/(2r)] [T]   [V_R]
    [1/r -L/(2r)] [R] = [V_L]
    or
    V_R=T/r+0.5*R*L/r
    V_L=T/r-0.5*R*L/r
    where r is the wheel radius and L is the robot's width.
    */
    double halfWidth = drivebaseWidth * 0.5;
    setVelocity(
      translate/wheelRadius + halfWidth*rotate/wheelRadius,
      translate/wheelRadius + halfWidth*rotate/wheelRadius
    );
  }
  /**Moves the wheels at absolute rotation speeds.
   * @param left rotation speed for left motors in radians/sec
   * @param right rotation speed for right motors in radians/sec
   * @author Ben
   * */
  public void setVelocity(double left,double right){
    //The conversion constant from the constructor is used here to convert an output rotation rate of the wheels
    //to the required units (encoder ticks/100ms).
    double leftAdjusted=left*rotationRateConversionConstant;
    double rightAdjusted=right*rotationRateConversionConstant;
    leftLeader.set(ControlMode.Velocity, leftAdjusted);
    rightLeader.set(ControlMode.Velocity, rightAdjusted);
  }
}

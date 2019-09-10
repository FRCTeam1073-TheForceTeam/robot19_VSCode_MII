/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
  public final boolean motorPowerSafety;
  public final double motorPowerLimit;

  public Drivetrain(double wheelRadius_, double drivebaseWidth_) {
    this(wheelRadius_, drivebaseWidth_, false, 1);
  }

  public Drivetrain(double wheelRadius_, double drivebaseWidth_, boolean motorPowerSafety_, double motorPowerLimit_) {
    wheelRadius = wheelRadius_;
    drivebaseWidth = drivebaseWidth_;
    motorPowerSafety = motorPowerSafety_;
    motorPowerLimit = motorPowerLimit_;
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
//    rightFollower.configFactoryDefault();
  //  rightFollower2.configFactoryDefault();
    leftLeader.configFactoryDefault();
 //   leftFollower.configFactoryDefault();
 //   leftFollower2.configFactoryDefault();

    /** Motor safety config */
    rightLeader.setSafetyEnabled(false);
  //  rightFollower.setSafetyEnabled(false);
  //  rightFollower2.setSafetyEnabled(false);
    leftLeader.setSafetyEnabled(false);
   // leftFollower.setSafetyEnabled(false);
 //   leftFollower2.setSafetyEnabled(false);

    /** Motor neutral mode config */
    rightLeader.setNeutralMode(NeutralMode.Brake);
//    rightFollower.setNeutralMode(NeutralMode.Brake);
//    rightFollower2.setNeutralMode(NeutralMode.Brake);
    leftLeader.setNeutralMode(NeutralMode.Brake);
//    leftFollower.setNeutralMode(NeutralMode.Brake);
//    leftFollower2.setNeutralMode(NeutralMode.Brake);

    /** Motor direction config */
    rightLeader.setInverted(true);
//    rightFollower.setInverted(true);
//    rightFollower2.setInverted(true);
    leftLeader.setInverted(false);
//    leftFollower.setInverted(false);
    //    leftFollower2.setInverted(false);

		leftLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 1000);
    rightLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 1000);

    leftLeader.config_kP(0, 10);
    leftLeader.config_kI(0, 0.1);
    leftLeader.config_kD(0, 350);
    rightLeader.config_kP(0, 10);
    rightLeader.config_kI(0, 0.1);
    rightLeader.config_kD(0, 350);
    /** Motor following config */
 //   rightFollower.follow(rightLeader);
  //  rightFollower2.follow(rightLeader);
//    leftFollower.follow(leftLeader);
//    leftFollower2.follow(leftLeader);
  }

  @Override
  public void initDefaultCommand() {
    
    /*setDefaultCommand(
    new ControlledMotorGroup(
      new ControlledMotor[] { 
        new ControlledMotorAnalog(leftLeader, 1, 0.5),
        new ControlledMotorAnalog(rightLeader, 5, 0.5),
      },
      this
    )
    );*/
    setDefaultCommand(new DriveControls());
  }
  
  @Override
  public void periodic() {
  SmartDashboard.putNumber("Left joystick Y", 100.0*Robot.oi.getDrAxis(1));
  SmartDashboard.putNumber("Right joystick Y", 100.0*Robot.oi.getDrAxis(5));
  SmartDashboard.putNumber("Left encoder power",leftLeader.getMotorOutputPercent());
  SmartDashboard.putNumber("right encoder power",rightLeader.getMotorOutputPercent());
  SmartDashboard.putNumber("Left encoder velocity",leftLeader.getSelectedSensorVelocity());
  SmartDashboard.putNumber("Right encoder velocity",rightLeader.getSelectedSensorVelocity());
  SmartDashboard.putNumber("Power",RobotMap.pdp.getVoltage());
  SmartDashboard.putNumber("Temperature",RobotMap.pdp.getTemperature());
  }
  
  /** Sets the motors to the input value */
  public void tankSet(double left, double right) {
    leftLeader.set(ControlMode.PercentOutput, left);
    rightLeader.set(ControlMode.PercentOutput, right);
  }

  /** Checks if the input is outside of the deadzone*/
  private double[] deadzoneFilter(double left, double right) {
    left = Utilities.deadzone(left);
    right = Utilities.deadzone(right);
    return new double[] { left, right };
  }

  /** This SHOULD MAYBE take both of the input values and produce movement that does not favor either input, but is far from final */
  public void arcadeDrive(double forward, double rotation) {

    /*
    //  Sorry, Jack.
    double mid, right, left;
    mid = forward - (Math.abs(rotation) * Math.signum(forward));
    if (rotation > 0) {
    right = mid - (Math.abs(rotation) * Math.signum(forward));
    left = mid + (Math.abs(rotation) * Math.signum(forward));
    } else {
    right = mid + (Math.abs(rotation) * Math.signum(forward));
    left = mid - (Math.abs(rotation) * Math.signum(forward));
    }*/
  double left,right;
  left = Utilities.clip(forward + rotation, 1);
  right = Utilities.clip(forward - rotation, 1);
  if (motorPowerSafety) {
    right *= motorPowerLimit;
    left *= motorPowerLimit;
  }
  double[] data = deadzoneFilter(left, right);
  left = data[0];
  right = data[1];
  tankSet(left, right);
  }
  
  /**Tank drive based on real speed and rotation rate (translate in cm/s, rotate in radians/s) 
  * WIP, obviously.
  */
  public void velocityDefinedDrive(double translate, double rotate) {
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
    setVelocity(translate / wheelRadius + halfWidth * rotate / wheelRadius,
        translate / wheelRadius + halfWidth * rotate / wheelRadius);
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
    System.out.println(leftAdjusted + "," + rightAdjusted);
  }
}

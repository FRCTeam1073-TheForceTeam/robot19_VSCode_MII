package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  
  /** If you are using multiple modules, make sure to define both the port
  * number and the module. For example you with a rangefinder:
  * public static int rangefinderPort = 1;
  * public static int rangefinderModule = 1;
  */
  
	public static ADXRS450_Gyro headingGyro;
	public static WPI_TalonSRX rightLeader;
	public static WPI_VictorSPX rightFollower;
	public static WPI_VictorSPX rightFollower2;
	public static WPI_TalonSRX leftLeader;
	public static WPI_VictorSPX leftFollower;
	public static WPI_VictorSPX leftFollower2;

	public static WPI_TalonSRX leftClimber;
	public static WPI_TalonSRX rightClimber;
	public static WPI_TalonSRX climbWheels;

	public static WPI_TalonSRX hatchCollect;
	public static WPI_TalonSRX hatchLift;
	public static WPI_VictorSPX hatchLiftFollower;

	public static Solenoid high;
	public static Solenoid low;

	public static BuiltInAccelerometer accelerometer;

	public static Solenoid fingerRaiser;
	public static Solenoid fingerLowerer;

	public static void init() {

		leftLeader = new WPI_TalonSRX(8);
		leftFollower = new WPI_VictorSPX(9);
		leftFollower2 = new WPI_VictorSPX(7);

		rightLeader = new WPI_TalonSRX(4);
		rightFollower = new WPI_VictorSPX(2);
		rightFollower2 = new WPI_VictorSPX(3);

		headingGyro = new ADXRS450_Gyro();
		accelerometer = new BuiltInAccelerometer();

		hatchCollect = new WPI_TalonSRX(5);
		hatchLift = new WPI_TalonSRX(6);
		hatchLiftFollower = new WPI_VictorSPX(12);

		rightClimber = new WPI_TalonSRX(10);
		leftClimber = new WPI_TalonSRX(13);
		climbWheels = new WPI_TalonSRX(15);

		high = new Solenoid(7);
		low = new Solenoid(6);
		fingerRaiser = new Solenoid(5);
		fingerLowerer = new Solenoid(4);
	}
}
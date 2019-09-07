package frc.robot;

public class Presets {
    public static double deadzone = 0.05;

    //Wheel radius (in cm)
    public final static double wheelRadius = 8; //Placeholder value

    //Width of the robot's drivebase (in cm)
    public final static double drivebaseWidth = 500; //Placeholder value

    public static double deadzoneFilter(double a){
        return deadzoneFilter(a,Presets.deadzone);
    }

    public static double deadzoneFilter(double a, double deadzone){
        if(Math.abs(a)<deadzone){
            return 0;
        } else {
            return Math.signum(a)*(Math.abs(a)-deadzone)*(1-deadzone);
        }
    }
}
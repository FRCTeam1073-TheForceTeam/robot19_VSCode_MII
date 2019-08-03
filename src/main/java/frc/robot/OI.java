package frc.robot;

import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * Use this class to assign functions to Xbox Controller buttons
 * 
 */

public class OI {

    /** Controllers */
    public XboxController driverControl, operatorControl;

    /** Driver Controls */
    public JoystickButton driverCancel;

    /** Operator Controls */
    public JoystickButton operatorCancel;

    public OI() {
        driverControl = new XboxController(0);
        operatorControl = new XboxController(1);

        driverCancel = driverControl.a;
        operatorCancel = operatorControl.a;

    }

    public double getDrAxis(int axis){
        return driverControl.getRawAxis(axis);
    }

    public double getOpAxis(int axis){
        return operatorControl.getRawAxis(axis);
    }
}
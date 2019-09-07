import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Presets;
import frc.robot.Robot;
import frc.robot.subsystems.*;

public class ClimberControls extends Command{
    public final int leftAxis=3;
    public final int rightAxis=6;
    public final JoystickButton invertButtonLeft=Robot.oi.operatorControl.leftBumper;
    public final JoystickButton invertButtonRight=Robot.oi.operatorControl.rightBumper;
    public HatchControls(){
        requires(Robot.climber);
    }
    @Override
    protected void execute(){
        double left=Presets.deadzone(Robot.oi.getOpAxis(leftAxis));
        double right=Presets.deadzone(Robot.oi.getOpAxis(rightAxis));
        left*=invertButtonLeft.get()?1:-1;
        right*=invertButtonRight.get()?1:-1;
        Robot.climber.setClimber(left, right);
    }
    @Override
    protected boolean isFinished(){
        return false;
    }
}
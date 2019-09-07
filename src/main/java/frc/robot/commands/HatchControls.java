import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.*;

public class HatchControls extends Command{
    public final int liftAxis=1;
    public final int intakeAxis=4;
    public HatchControls(){
        requires(Robot.hatch);
    }
    @Override
    protected void execute(){
        Robot.hatch.setLift(deadzone(Robot.oi.getOpAxis(liftAxis)));
        Robot.hatch.setIntake(deadzone(Robot.oi.getOpAxis(intakeAxis)));
    }
    @Override
    protected boolean isFinished(){
        return false;
    }
}
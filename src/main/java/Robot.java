import Constants.CTREConfigs;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    public static CTREConfigs ctreConfigs;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        ctreConfigs = new CTREConfigs();
        robotContainer = new RobotContainer();
    }
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousPeriodic(){

    }

    @Override
    public void teleopPeriodic(){

    }
}

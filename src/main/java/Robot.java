import Constants.CTREConfigs;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    public static CTREConfigs ctreConfigs;
    private Command autonomous;

    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        ctreConfigs = new CTREConfigs();
        robotContainer = new RobotContainer();
        autonomous = new Autonomous(robotContainer.swerve, "paths/Example path.path");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        if(autonomous != null) {
            autonomous.schedule();
        }
    }

    @Override
    public void autonomousPeriodic(){}

    @Override
    public void teleopInit() {
        if(autonomous != null) {
            autonomous.cancel();
        }
    }

    @Override
    public void teleopPeriodic(){

    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }
}

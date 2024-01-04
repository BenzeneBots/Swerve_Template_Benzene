import Constants.RobotMap;
import Constants.SwerveConstants;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
    private final Joystick driver = new Joystick(0);
    private final int translationAxis = RobotMap.defaultY;
    private final int strafeAxis = RobotMap.defaultX;
    private final int rotationAxis = RobotMap.defaultZ;
    private final int invertedAxis = RobotMap.invertedAxis;

    public final Drivetrain swerve = new Drivetrain(true);

    public RobotContainer() {
        swerve.setDefaultCommand(
                new TeleOpSwerve(
                        swerve,
                        driver.getRawAxis(translationAxis),
                        driver.getRawAxis(strafeAxis),
                        driver.getRawAxis(rotationAxis),
                        driver.getRawAxis(invertedAxis),
                        false,
                        new JoystickButton(driver, RobotMap.slowMode),
                        new JoystickButton(driver, RobotMap.resetGyro)
                )
        );
    }
}

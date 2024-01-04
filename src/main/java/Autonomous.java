import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Autonomous extends SequentialCommandGroup {
    Drivetrain swerve;
    public Autonomous(Drivetrain swerve, String pathName) {
        this.swerve = swerve;
        addRequirements(swerve);
        addCommands(
                new PathFollower(swerve, pathName)
        );
    }
}

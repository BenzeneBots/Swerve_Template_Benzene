import Constants.SwerveConstants;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

public class PathFollower extends CommandBase {
    private HolonomicDriveController holonomicDriveController;
    private PathPlannerTrajectory.PathPlannerState state;
    private Pose2d currentPose;
    private ChassisSpeeds speeds = new ChassisSpeeds();
    private PathPlannerTrajectory path;
    private ProfiledPIDController rotPID;
    Drivetrain swerve;

    private final Timer timer = new Timer();

    public PathFollower(Drivetrain swerve, String pathName) {
        this.rotPID = SwerveConstants.follower.ROT_PID_CONTROLLER;
        this.swerve = swerve;
        this.path = PathPlanner.loadPath(pathName, SwerveConstants.maxSpeedMPS, SwerveConstants.maxAcceleration);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        this.rotPID.enableContinuousInput(-Math.PI, Math.PI);
        holonomicDriveController = new HolonomicDriveController(SwerveConstants.follower.X_PID_CONTROLLER, SwerveConstants.follower.Y_PID_CONTROLLER, this.rotPID);

        timer.stop();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        var currTime = timer.get();
        state = (PathPlannerTrajectory.PathPlannerState) path.sample(currTime);
        currentPose = swerve.getPose();
        speeds = holonomicDriveController.calculate(currentPose, state, state.holonomicRotation);
        swerve.updateStates(SwerveConstants.swerveKinematics.toSwerveModuleStates(speeds));
    }

    public double getTime() {return timer.get();}

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {
        swerve.updateStates(SwerveConstants.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds()));
        timer.stop();
    }
}

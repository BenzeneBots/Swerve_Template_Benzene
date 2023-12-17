package team4384.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team4384.robot.RobotContainer;
import team4384.robot.constants.SwerveMap;

import java.util.List;

public class Autonomous {
    private Swerve s_Swerve;
    private AHRS gyro;
    private Trajectory trajectory;
    public Autonomous(Swerve m_swerve, AHRS gyro, Trajectory trajectory) {
        this.s_Swerve = m_swerve;
        this.gyro = gyro;
        this.trajectory = trajectory;
    }
    private static double microTime(double seconds) {
        return seconds * 1000000;
    }

    private void follower() {
        /* Template for Trajectory
{
  "acceleration": 10.0,
  "curvature": 0.0,
  "pose": {
    "rotation": {
      "radians": 0.0
    },
    "translation": {
      "x": 1.4112459963428536,
      "y": 4.999200004486452
    }
  },
  "time": 0.0,
  "velocity": 0.0
},
         */
        List<Trajectory.State> states = this.trajectory.getStates();
        for(int i = 0; i < states.size(); i++) {
            long initTime = RobotController.getFPGATime();
            Trajectory.State current = states.get(i);
            double time_diff = 0.1;
            Trajectory.State next;
            if(i != states.size() - 1) {
                next = states.get(i + 1);
                time_diff = next.timeSeconds - current.timeSeconds;
            }
            while((RobotController.getFPGATime() - initTime) < microTime(time_diff)) {
                s_Swerve.drive(
                        current.poseMeters.getTranslation(),
                        current.poseMeters.getRotation().getRadians(),
                        false,
                        false
                );
            }
        }
    }

    public void basic() {
    }
}

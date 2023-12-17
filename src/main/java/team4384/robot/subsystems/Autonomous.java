package team4384.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team4384.robot.constants.SwerveMap;

public class Autonomous {
    private Swerve s_Swerve;
    private AHRS gyro;
    public Autonomous(Swerve m_swerve, AHRS gyro) {
        this.s_Swerve = m_swerve;
        this.gyro = gyro;
    }
    private static double microTime(double seconds) {
        return seconds * 1000000;
    }
    public void basic() {
    }
}

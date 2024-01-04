package Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class SwerveConstants {

    public static final double trackWidth = Units.inchesToMeters(20);
    public static final double wheelBase = Units.inchesToMeters(23);

    public static final boolean invertGyro = true;

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    public static final double maxTeleSpeed = 0.63;
    public static final double maxSlowTeleSpeed = 0.2;
    /** Radians per Second */
    public static final double maxAngularVelocity = 0.5;
    public static final double maxRotSpeed = .3;

    public static double maxSpeedMPS = 0.2;

    public static final FalconSwerveConstants chosenModule = FalconSwerveConstants.SDSMK4i(FalconSwerveConstants.driveGearRatios.SDSMK4i_L2);
    public static double gearRatio = FalconSwerveConstants.driveGearRatios.SDSMK4i_L2;
    public static double wheelCircumference = chosenModule.wheelCircumference;

    // Motor Configs
    public static boolean turnMotorInvert = true;
    public static boolean driveMotorInvert = true;

    public static NeutralMode driveMotorNeutral = NeutralMode.Coast;
    public static NeutralMode turnMotorNeutral = NeutralMode.Brake;

    public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;
    public static final double angleKF = chosenModule.angleKF;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    public static int FL0drive = 0;
    public static int FL0turn = 1;
    public static int FL0encoder = 2;
    public static Rotation2d FL0offset = new Rotation2d(0.0);
    public static int FR1drive = 0;
    public static int FR1turn = 1;
    public static int FR1encoder = 2;
    public static Rotation2d FR1offset = new Rotation2d(0.0);
    public static int BL2drive = 0;
    public static int BL2turn = 1;
    public static int BL2encoder = 2;
    public static Rotation2d BL2offset = new Rotation2d(0.0);
    public static int BR3drive = 0;
    public static int BR3turn = 1;
    public static int BR3encoder = 2;
    public static Rotation2d BR3offset = new Rotation2d(0.0);

}

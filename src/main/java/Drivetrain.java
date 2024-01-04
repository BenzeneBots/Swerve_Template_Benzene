import Constants.SwerveConstants;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] swerveModules;
    public final AHRS gyro;
    private ShuffleboardTab driveTrainTab;
    private GenericEntry[] CanCoder = {};
    private GenericEntry[] Velocity;
    private GenericEntry[] Angle;
    private SwerveModuleState[] desiredStates;
    private boolean isOpenLoop;

    public Drivetrain(boolean isOpenLoop) {
        this.isOpenLoop = isOpenLoop;

        gyro = new AHRS(SerialPort.Port.kUSB);
        zeroGyro();

        this.driveTrainTab = Shuffleboard.getTab("Drive Train");

        this.swerveModules = new SwerveModule[] {
                new SwerveModule(0, "FL", SwerveConstants.FL0drive, SwerveConstants.FL0turn, SwerveConstants.FL0encoder, SwerveConstants.FL0offset),
                new SwerveModule(1, "FR", SwerveConstants.FR1drive, SwerveConstants.FR1turn, SwerveConstants.FR1encoder, SwerveConstants.FR1offset),
                new SwerveModule(2, "BL", SwerveConstants.BL2drive, SwerveConstants.BL2turn, SwerveConstants.BL2encoder, SwerveConstants.BL2offset),
                new SwerveModule(3, "BR", SwerveConstants.BR3drive, SwerveConstants.BR3turn, SwerveConstants.BR3encoder, SwerveConstants.BR3offset),
        };

        Timer.delay(1.0); // Necessary?
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, getYaw(), getModulePosition());

        this.CanCoder = new GenericEntry[]{
                driveTrainTab.add(swerveModules[0].moduleName + swerveModules[0].moduleNumber + " Cancoder", swerveModules[0].getCanCoder().getDegrees()).getEntry(),
                driveTrainTab.add(swerveModules[1].moduleName + swerveModules[1].moduleNumber + " Cancoder", swerveModules[1].getCanCoder().getDegrees()).getEntry(),
                driveTrainTab.add(swerveModules[2].moduleName + swerveModules[2].moduleNumber + " Cancoder", swerveModules[2].getCanCoder().getDegrees()).getEntry(),
                driveTrainTab.add(swerveModules[3].moduleName + swerveModules[3].moduleNumber + " Cancoder", swerveModules[3].getCanCoder().getDegrees()).getEntry()
        };

        this.Angle = new GenericEntry[]{
                driveTrainTab.add(swerveModules[0].moduleName + swerveModules[0].moduleNumber + " Integrated", swerveModules[0].getAngle().getDegrees()).getEntry(),
                driveTrainTab.add(swerveModules[1].moduleName + swerveModules[1].moduleNumber + " Integrated", swerveModules[1].getAngle().getDegrees()).getEntry(),
                driveTrainTab.add(swerveModules[2].moduleName + swerveModules[2].moduleNumber + " Integrated", swerveModules[2].getAngle().getDegrees()).getEntry(),
                driveTrainTab.add(swerveModules[3].moduleName + swerveModules[3].moduleNumber + " Integrated", swerveModules[3].getAngle().getDegrees()).getEntry()
        };

        this.Velocity = new GenericEntry[]{
                driveTrainTab.add(swerveModules[0].moduleName + swerveModules[0].moduleNumber + " Velocity", swerveModules[0].getSpeed()).getEntry(),
                driveTrainTab.add(swerveModules[1].moduleName + swerveModules[1].moduleNumber + " Velocity", swerveModules[1].getSpeed()).getEntry(),
                driveTrainTab.add(swerveModules[2].moduleName + swerveModules[2].moduleNumber + " Velocity", swerveModules[2].getSpeed()).getEntry(),
                driveTrainTab.add(swerveModules[3].moduleName + swerveModules[3].moduleNumber + " Velocity", swerveModules[3].getSpeed()).getEntry()
        };
    }

    public SwerveModulePosition[] getModulePosition() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : swerveModules) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.zeroYaw();
    }

    public Rotation2d getYaw() {
        return (SwerveConstants.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute() {
        for(SwerveModule mod : swerveModules) {
            mod.resetToAbsolute();
        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void updateStates(SwerveModuleState[] states) {
        this.desiredStates = states;
    }

    public void updateOdometery() {
        swerveOdometry.update(Rotation2d.fromDegrees(gyro.getYaw()), getModulePosition());
    }

    @Override
    public void periodic() {
        for(int i = 0; i < swerveModules.length; i++) {
            SwerveModule mod = swerveModules[i];

            CanCoder[i].setDouble(mod.getCanCoder().getDegrees());
            Angle[i].setDouble(mod.getPosition().angle.getDegrees());
            Velocity[i].setDouble(mod.getSpeed());
        }

        for(SwerveModule mod : swerveModules) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], isOpenLoop);
        }

        updateOdometery();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
                SwerveConstants.swerveKinematics.toSwerveModuleStates(
                        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation,
                                gyro.getRotation2d()
                        ) :
                                new ChassisSpeeds(
                                        translation.getX(),
                                        translation.getY(),
                                        rotation
                                )
                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeedMPS);

        this.desiredStates = swerveModuleStates;
    }
}

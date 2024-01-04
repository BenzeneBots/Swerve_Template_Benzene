import Constants.RobotMap;
import Constants.SwerveConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleOpSwerve extends CommandBase {
    private final Drivetrain s_Swerve;
    private final double translationSup;
    private final double strafeSup;
    private final double rotationSup;
    private final boolean robotCentricSup;
    private final JoystickButton SlowMode;
    private final JoystickButton resetGyro;
    private final double Inverted;

    public TeleOpSwerve(Drivetrain s_Swerve, double translationSup, double strafeSup, double rotationSup, double Inverted, boolean robotCentricSup, JoystickButton SlowMode, JoystickButton resetGyro) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.SlowMode = SlowMode;
        this.resetGyro = resetGyro;
        this.Inverted = Inverted;
    }
    @Override
    public void execute() {
        double translationVal = 0;
        double strafeVal = 0;
        double rotationVal = 0;

        if (SlowMode.getAsBoolean()) {
            translationVal = -MathUtil.applyDeadband(MathUtil.clamp(translationSup, -SwerveConstants.maxSlowTeleSpeed, SwerveConstants.maxSlowTeleSpeed), RobotMap.stickDeadband);
            strafeVal = -MathUtil.applyDeadband(MathUtil.clamp(strafeSup, -SwerveConstants.maxSlowTeleSpeed, SwerveConstants.maxSlowTeleSpeed), RobotMap.stickDeadband);

        }
        else  {
            translationVal = MathUtil.applyDeadband(MathUtil.clamp(translationSup, -SwerveConstants.maxTeleSpeed, SwerveConstants.maxTeleSpeed), RobotMap.stickDeadband) * -1;
            strafeVal = MathUtil.applyDeadband(MathUtil.clamp(strafeSup, -SwerveConstants.maxTeleSpeed, SwerveConstants.maxTeleSpeed), RobotMap.stickDeadband) * -1;
        }
        rotationVal = MathUtil.applyDeadband(MathUtil.clamp(rotationSup, -SwerveConstants.maxRotSpeed , SwerveConstants.maxRotSpeed), RobotMap.stickDeadband);

        double Invert = Inverted;

        if (Invert == 1 ||Invert == -1) {
            translationVal *= Invert;
            strafeVal *= Invert;
        }

        if (resetGyro.getAsBoolean()) {
            s_Swerve.zeroGyro();
        }

        /* Drive */
        s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeedMPS),
                rotationVal * SwerveConstants.maxAngularVelocity,
                !robotCentricSup,
                true
        );
    }
}

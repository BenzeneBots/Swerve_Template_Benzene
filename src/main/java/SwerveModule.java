import Constants.SwerveConstants;
import Conversions.SwerveConversions;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import util.CTREModuleState;

public class SwerveModule {
    public int moduleNumber;
    public String moduleName;
    // We'll see if we need this
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private TalonFX driveMotor;
    private TalonFX turnMotor;
    private CANCoder angleEncoder;

    public SwerveModule(int moduleNumber, String moduleName, int driveMotor, int turnMotor, int encoderID, Rotation2d angleOffset) {
        this.moduleName = moduleName;
        this.moduleNumber = moduleNumber;
        this.angleOffset = angleOffset;

        this.driveMotor = new TalonFX(driveMotor);
        this.turnMotor = new TalonFX(turnMotor);
        this.angleEncoder = new CANCoder(driveMotor);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = CTREModuleState.optimize(desiredState, getAngle());
        setAngle(desiredState);
        setSpeed(isOpenLoop, desiredState);
    }

    private void setSpeed(boolean isOpenLoop, SwerveModuleState desiredState) {
        if(isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.maxSpeedMPS;
            driveMotor.set(TalonFXControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = SwerveConversions.MPSToFalcon(desiredState.speedMetersPerSecond, SwerveConstants.wheelCircumference, SwerveConstants.gearRatio);
            driveMotor.set(TalonFXControlMode.Velocity, velocity);
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.maxSpeedMPS * 0.01)) ? lastAngle : desiredState.angle;

        turnMotor.set(TalonFXControlMode.Position, SwerveConversions.DegreesToFalcon(angle.getDegrees(), SwerveConstants.gearRatio));
        lastAngle = angle;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(SwerveConversions.FalconToDegrees(turnMotor.getSelectedSensorPosition(), SwerveConstants.gearRatio));
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute() {
        double absolutePosition = SwerveConversions.DegreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), SwerveConstants.gearRatio);
        turnMotor.setSelectedSensorPosition(absolutePosition);
    }

    // Add Configs
    private void configAngleEncoder(){
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        turnMotor.configFactoryDefault();
        turnMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        turnMotor.setInverted(SwerveConstants.turnMotorInvert);
        turnMotor.setNeutralMode(SwerveConstants.turnMotorNeutral);
        resetToAbsolute();
    }

    private void configDriveMotor(){
        driveMotor.configFactoryDefault();
        driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        driveMotor.setInverted(SwerveConstants.driveMotorInvert);
        driveMotor.setNeutralMode(SwerveConstants.driveMotorNeutral);
        driveMotor.setSelectedSensorPosition(0);
    }

    public double getSpeed() {
        return SwerveConversions.FalconToMPS(driveMotor.getSelectedSensorPosition(), SwerveConstants.wheelCircumference, SwerveConstants.gearRatio);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                SwerveConversions.FalconToMeters(driveMotor.getSelectedSensorPosition(), SwerveConstants.wheelCircumference, SwerveConstants.gearRatio),
                getAngle()
        );
    }
}

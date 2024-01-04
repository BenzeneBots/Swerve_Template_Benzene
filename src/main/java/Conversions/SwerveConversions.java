package Conversions;

public class SwerveConversions {
    public static double FalconToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / (gearRatio * 2048.0));
    }

    public static double DegreesToFalcon(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 2048.0));
    }

    public static double RPMToFalcon(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        return (motorRPM * (2048.0 / 600.0));
    }

    public static double FalconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);
        return motorRPM / gearRatio;
    }

    public static double FalconToMPS (double velocityCounts, double circumference, double gearRatio) {
        double wheelRPM = FalconToRPM(velocityCounts, gearRatio);
        return (wheelRPM * circumference) / 60;
    }

    public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
        double wheelRPM = ((velocity * 60) / circumference);
        return RPMToFalcon(wheelRPM, gearRatio);
    }

    public static double FalconToMeters(double positionCounts, double circumference, double gearRatio) {
        return positionCounts * (circumference / (gearRatio * 2048.0));
    }

}

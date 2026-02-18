package frc.robot;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;

public class GeometryUtil {

    public static class SphericalCoordinate {
        double magnitude;
        Angle pitch;
        Angle yaw;

        public SphericalCoordinate(double magnitude, Angle pitch, Angle yaw) {
            this.magnitude = magnitude;
            this.pitch = pitch;
            this.yaw = yaw;
        }
    }

    public double degreesToRadians(double angle) {
        return angle * Math.PI / 180;
    }

    public double radiansToDegrees(double angle) {
        return angle * 180 / Math.PI;
    }

    public static Translation3d sphericalToCartesian(SphericalCoordinate target) {
        // System.out.printf("magnitude: %g, pitch: %g, yaw: %g \n", target.magnitude,
        // target.pitch.in(Degrees),
        // target.yaw.in(Degrees));
        return new Translation3d(
                target.magnitude * Math.cos(target.pitch.in(Radians)) * Math.cos(target.yaw.in(Radians)),
                target.magnitude * Math.cos(target.pitch.in(Radians)) * Math.sin(target.yaw.in(Radians)),
                target.magnitude * Math.sin(target.pitch.in(Radians)));

    }

    public static SphericalCoordinate cartesianToSpherical(double x, double y, double z) {
        double magnitude = Math.sqrt(x * x + y * y + z * z);
        double yaw = Math.atan2(y, x);
        double pitch = Math.atan2(z, Math.sqrt(x * x + y * y));
        return new SphericalCoordinate(magnitude, Radians.of(pitch), Radians.of(yaw));
    }
}

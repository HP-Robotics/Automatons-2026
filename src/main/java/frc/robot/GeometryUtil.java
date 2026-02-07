package frc.robot;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;

public class GeometryUtil {

    public class SphericalCoordinate {
        double magnitude;
        Angle pitch; // radians
        Angle yaw; // radians        

        public SphericalCoordinate(double magnitude, Angle pitch, Angle yaw) {
            this.magnitude = magnitude;
            this.pitch = pitch;
            this.yaw = pitch;
        }
    }

    public double degreesToRadians(double angle) {
        return angle * Math.PI / 180;
    }

    public double radiansToDegrees(double angle) {
        return angle * 180 / Math.PI;
    }

    public Translation3d sphericalToCartesian(SphericalCoordinate target) {
        return new Translation3d(target.magnitude,
                new Rotation3d(0, target.pitch.in(Radians), target.yaw.in(Radians)));
    }

    public SphericalCoordinate cartesianToSpherical(double x, double y, double z) {
        double magnitude = Math.sqrt(x * x + y * y + z * z);
        double yaw = Math.atan2(y, x);
        double pitch = Math.atan2(z, Math.sqrt(x * x + y * y));
        return new SphericalCoordinate(magnitude, Radians.of(pitch), Radians.of(yaw));
    }
}

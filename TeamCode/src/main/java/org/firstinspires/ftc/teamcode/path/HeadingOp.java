package org.firstinspires.ftc.teamcode.path;

import org.apache.commons.math3.util.FastMath;

import org.firstinspires.ftc.teamcode.util.MathHelper;
import org.firstinspires.ftc.teamcode.util.Pose;

/// Controls the robot's target heading along a path.
@FunctionalInterface
public interface HeadingOp {

    double DEFAULT_EXPONENTIAL_RATE = 4;

    /// @return the target heading in radians
    /// @param progress progress along the path (0 to 1)
    /// @param x position on the path here, inches
    /// @param y position on the path here, inches
    /// @param tangentAngle direction of the path tangent at this point in radians
    /// @param reversed whether the path is flagged as reversed
    /// @return the target heading in radians
    double heading(double progress, double x, double y, double tangentAngle, boolean reversed);

    static HeadingOp constantHeading(double heading) {
        return (progress, x, y, tangentAngle, reversed) -> heading;
    }

    static HeadingOp linearHeading(double startHeading, double endHeading) {
        double delta = MathHelper.normalizeAngleRad(endHeading - startHeading);
        return (progress, x, y, tangentAngle, reversed) ->
                MathHelper.normalizeAngleRad(startHeading + delta * progress);
    }

    static HeadingOp linearHeadingReflex(double startHeading, double endHeading) {
        double delta = reflex(MathHelper.normalizeAngleRad(endHeading - startHeading));
        return (progress, x, y, tangentAngle, reversed) ->
                MathHelper.normalizeAngleRad(startHeading + delta * progress);
    }

    static HeadingOp exponentialHeading(double startHeading, double endHeading) {
        return exponentialHeading(startHeading, endHeading, DEFAULT_EXPONENTIAL_RATE);
    }

    static HeadingOp exponentialHeading(double startHeading, double endHeading, double rate) {
        double delta = MathHelper.normalizeAngleRad(endHeading - startHeading);
        double denom = 1 - Math.exp(-rate);
        return (progress, x, y, tangentAngle, reversed) -> {
            double ease = (1 - Math.exp(-rate * progress)) / denom;
            return MathHelper.normalizeAngleRad(startHeading + delta * ease);
        };
    }

    static HeadingOp exponentialHeadingReflex(double startHeading, double endHeading) {
        return exponentialHeadingReflex(startHeading, endHeading, DEFAULT_EXPONENTIAL_RATE);
    }

    static HeadingOp exponentialHeadingReflex(double startHeading, double endHeading, double rate) {
        double delta = reflex(MathHelper.normalizeAngleRad(endHeading - startHeading));
        double denom = 1 - Math.exp(-rate);
        return (progress, x, y, tangentAngle, reversed) -> {
            double ease = (1 - Math.exp(-rate * progress)) / denom;
            return MathHelper.normalizeAngleRad(startHeading + delta * ease);
        };
    }

    static HeadingOp tangentialHeading() {
        return (progress, x, y, tangentAngle, reversed) ->
                MathHelper.normalizeAngleRad(tangentAngle + (reversed ? Math.PI : 0));
    }

    static HeadingOp facePoint(double pointX, double pointY) {
        return (progress, x, y, tangentAngle, reversed) -> {
            double toPoint = FastMath.atan2(pointY - y, pointX - x);
            return MathHelper.normalizeAngleRad(toPoint + (reversed ? Math.PI : 0));
        };
    }

    static HeadingOp facePoint(Pose point) {
        return facePoint(point.x, point.y);
    }

    static HeadingOp customHeading(HeadingOp op) {
        return op;
    }

    static double reflex(double shortDelta) { //follows the reflex angle

        if (shortDelta > 0) return shortDelta - 2 * Math.PI;
        if (shortDelta < 0) return shortDelta + 2 * Math.PI;
        return 2 * Math.PI;
    }
}

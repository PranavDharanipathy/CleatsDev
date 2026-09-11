package org.firstinspires.ftc.teamcode.path;

import org.apache.commons.math3.util.FastMath;

import org.firstinspires.ftc.teamcode.util.MathHelper;
import org.firstinspires.ftc.teamcode.util.Pose;

/// Controls the robot's target heading along a path.
@FunctionalInterface
public interface HeadingOp {

    double DEFAULT_EXPONENTIAL_RATE = 4;

    double SLICE_EPSILON = 1e-6;

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

        return (progress, x, y, tangentAngle, reversed) -> {
            double ease = (1 - Math.exp(-rate * progress)) / (1 - Math.exp(-rate));
            return MathHelper.normalizeAngleRad(startHeading + delta * ease);
        };
    }

    static HeadingOp exponentialHeadingReflex(double startHeading, double endHeading) {
        return exponentialHeadingReflex(startHeading, endHeading, DEFAULT_EXPONENTIAL_RATE);
    }

    static HeadingOp exponentialHeadingReflex(double startHeading, double endHeading, double rate) {

        double delta = reflex(MathHelper.normalizeAngleRad(endHeading - startHeading));

        return (progress, x, y, tangentAngle, reversed) -> {
            double ease = (1 - Math.exp(-rate * progress)) / (1 - Math.exp(-rate));
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

    /// One part of a compound heading plan.
    class Slice {

        final double start, end;
        final HeadingOp op;

        private Slice(double start, double end, HeadingOp op) {

            this.start = start;
            this.end = end;
            this.op = op;
        }
    }

    /// @param startPercent where along the path this heading control takes over, 0 to 1
    /// @param endPercent where it hands the path over to the next slice, 0 to 1
    static Slice headingPart(double startPercent, double endPercent, HeadingOp op) {
        return new Slice(startPercent, endPercent, op);
    }

    /// Runs a different heading control over each slice of the path. Slices are given in
    /// order, must meet exactly, and must together cover the path from 0 to 1.
    static HeadingOp compound(Slice... headingOpSlices) {

        if (headingOpSlices.length == 0) throw new IllegalArgumentException("A compound heading needs at least one part!");

        for (int i = 0; i < headingOpSlices.length; i++) {

            if (headingOpSlices[i].end <= headingOpSlices[i].start) throw new IllegalArgumentException("Heading part " + i + " ends at or before it starts!");

            if (i > 0 && Math.abs(headingOpSlices[i].start - headingOpSlices[i - 1].end) > SLICE_EPSILON) throw new IllegalArgumentException("Heading part " + i + " must start where part " + (i - 1) + " ended!");
        }

        if (Math.abs(headingOpSlices[0].start) > SLICE_EPSILON || Math.abs(headingOpSlices[headingOpSlices.length - 1].end - 1) > SLICE_EPSILON) throw new IllegalArgumentException("A compound heading must cover the whole path, 0 to 1!");

        Slice[] slices = headingOpSlices.clone();

        return (progress, x, y, tangentAngle, reversed) -> {

            double along = MathHelper.clamp(progress, 0, 1);

            for (Slice slice : slices) {
                if (along <= slice.end) return slice.op.heading(
                        MathHelper.clamp((along - slice.start) / (slice.end - slice.start), 0, 1),
                        x,
                        y,
                        tangentAngle,
                        reversed
                );
            }

            Slice last = slices[slices.length - 1];
            return last.op.heading(1, x, y, tangentAngle, reversed);
        };
    }

    static double reflex(double shortDelta) { //follows the reflex angle

        if (shortDelta > 0) return shortDelta - 2d * Math.PI;
        if (shortDelta < 0) return shortDelta + 2d * Math.PI;
        return 2d * Math.PI;
    }
}

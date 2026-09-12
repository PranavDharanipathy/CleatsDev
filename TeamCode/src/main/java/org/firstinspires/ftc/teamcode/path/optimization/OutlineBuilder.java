package org.firstinspires.ftc.teamcode.path.optimization;

import org.apache.commons.math3.util.FastMath;

import org.firstinspires.ftc.teamcode.util.MathHelper;

import java.util.ArrayList;
import java.util.function.Function;

public class OutlineBuilder<T> {

    //how far a traced curve can sit from the real one in inches
    private static final double ARC_TOLERANCE = 0.1;

    private static final int MAX_ARC_STEPS = 180;

    private final ArrayList<CollisionPoint> corners = new ArrayList<>();
    private final Function<CollisionPoint[], T> finish;

    public OutlineBuilder(double x, double y, Function<CollisionPoint[], T> finish) {

        this.finish = finish;
        corners.add(new CollisionPoint(x, y));
    }

    /// A straight edge from wherever the trace is now to here.
    public OutlineBuilder<T> lineTo(double x, double y) {

        corners.add(new CollisionPoint(x, y));

        return this;
    }

    /// @param sweepAngleRad how far around to swing in radians, positive being counter-clockwise
    public OutlineBuilder<T> arcTo(double centerX, double centerY, double sweepAngleRad) {
        return arcTo(centerX, centerY, sweepAngleRad, 0);
    }

    /// @param steps how many straight pieces trace the curve (or 0 to pick enough automatically)
    public OutlineBuilder<T> arcTo(double centerX, double centerY, double sweepAngleRad, int steps) {

        CollisionPoint from = corners.get(corners.size() - 1);

        double radius = Math.hypot(from.x - centerX, from.y - centerY);

        if (radius <= 0) throw new IllegalArgumentException("An arc cannot be swung around the point it starts on!");
        if (sweepAngleRad == 0) return this;

        double startAngle = FastMath.atan2(from.y - centerY, from.x - centerX);
        int pieces = steps > 0 ? steps : arcSteps(radius, sweepAngleRad);

        final double tangentRadius = radius / FastMath.cos(sweepAngleRad / (2d * pieces));

        for (int i = 1; i <= pieces; i++) {

            double angle = startAngle + sweepAngleRad * i / pieces;
            double reach = i == pieces ? radius : tangentRadius;

            corners.add(new CollisionPoint(centerX + reach * FastMath.cos(angle), centerY + reach * FastMath.sin(angle)));
        }

        return this;
    }

    /// @param points the x then the y of each point in turn
    public OutlineBuilder<T> curveThrough(double... points) {

        if (points.length % 2 != 0) throw new IllegalArgumentException("A curve needs an x and a y for every point it runs through!");

        //centripetal Catmull-Rom splines for the win!

        ArrayList<CollisionPoint> knots = new ArrayList<>();
        knots.add(corners.get(corners.size() - 1));

        for (int i = 0; i < points.length; i += 2) {

            CollisionPoint from = knots.get(knots.size() - 1);

            if (Math.hypot(points[i] - from.x, points[i + 1] - from.y) > 1e-9)
                knots.add(new CollisionPoint(points[i], points[i + 1]));
        }

        final int count = knots.size();

        if (count < 2) return this;
        if (count == 2) return lineTo(knots.get(1).x, knots.get(1).y);

        //mirrored end points keep the curve aligned with the given points.
        double[] x = new double[count + 2], y = new double[count + 2];

        for (int i = 0; i < count; i++) {
            x[i + 1] = knots.get(i).x;
            y[i + 1] = knots.get(i).y;
        }

        x[0] = 2 * x[1] - x[2];
        y[0] = 2 * y[1] - y[2];
        x[count + 1] = 2 * x[count] - x[count - 1];
        y[count + 1] = 2 * y[count] - y[count - 1];

        for (int i = 1; i < count; i++) curvePiece(x, y, i);

        return this;
    }

    private void curvePiece(double[] x, double[] y, int i) {

        //centripetal spacing is what keeps clustered points from throwing a loop rather than a bend
        double before = Math.sqrt(Math.hypot(x[i] - x[i - 1], y[i] - y[i - 1]));
        double across = Math.sqrt(Math.hypot(x[i + 1] - x[i], y[i + 1] - y[i]));
        double after = Math.sqrt(Math.hypot(x[i + 2] - x[i + 1], y[i + 2] - y[i + 1]));

        double spanX = x[i + 1] - x[i], spanY = y[i + 1] - y[i];

        double leaveX = spanX + across * ((x[i] - x[i - 1]) / before - (x[i + 1] - x[i - 1]) / (before + across));
        double leaveY = spanY + across * ((y[i] - y[i - 1]) / before - (y[i + 1] - y[i - 1]) / (before + across));
        double arriveX = spanX + across * ((x[i + 2] - x[i + 1]) / after - (x[i + 2] - x[i]) / (across + after));
        double arriveY = spanY + across * ((y[i + 2] - y[i + 1]) / after - (y[i + 2] - y[i]) / (across + after));

        double cubeX = leaveX + arriveX - 2 * spanX, cubeY = leaveY + arriveY - 2 * spanY;
        double squareX = 3 * spanX - 2 * leaveX - arriveX, squareY = 3 * spanY - 2 * leaveY - arriveY;

        int pieces = curveSteps(squareX, squareY, cubeX, cubeY);

        for (int piece = 1; piece < pieces; piece++) {

            double step = (double) piece / pieces;

            corners.add(new CollisionPoint(
                    ((cubeX * step + squareX) * step + leaveX) * step + x[i],
                    ((cubeY * step + squareY) * step + leaveY) * step + y[i]
            ));
        }

        corners.add(new CollisionPoint(x[i + 1], y[i + 1])); //goes through the point itself
    }

    private static int curveSteps(double squareX, double squareY, double cubeX, double cubeY) {

        double bend = Math.max(Math.hypot(squareX, squareY), Math.hypot(squareX + 3 * cubeX, squareY + 3 * cubeY));

        return MathHelper.clamp((int) Math.ceil(Math.sqrt(bend / (4 * ARC_TOLERANCE))), 1, MAX_ARC_STEPS);
    }

    private static int arcSteps(double radius, double sweepAngle) {

        if (radius <= ARC_TOLERANCE) return 1;

        double step = 2d * Math.acos(1d - ARC_TOLERANCE / radius);

        return MathHelper.clamp((int) Math.ceil(Math.abs(sweepAngle) / step), 1, MAX_ARC_STEPS);
    }

    public T build() {

        while (corners.size() > 1) {

            CollisionPoint first = corners.get(0), last = corners.get(corners.size() - 1);

            if (Math.hypot(last.x - first.x, last.y - first.y) > 1e-9) break;

            corners.remove(corners.size() - 1);
        }

        CollisionPoint[] traced = corners.toArray(new CollisionPoint[0]);

        if (traced.length > 2 && !Outline.isSimple(traced)) throw new IllegalArgumentException("This trace crosses over itself, it doesn't make one outline!");

        return finish.apply(traced);
    }
}

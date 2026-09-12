package org.firstinspires.ftc.teamcode.path.optimization;


import org.firstinspires.ftc.teamcode.following.chassis.MecanumProfile;
import org.firstinspires.ftc.teamcode.path.HeadingOp;
import org.firstinspires.ftc.teamcode.path.HermiteSpline;
import org.firstinspires.ftc.teamcode.path.Movement;
import org.firstinspires.ftc.teamcode.util.MathHelper;
import org.firstinspires.ftc.teamcode.util.Pose;

import java.util.ArrayList;

/// Builds the quickest path between two poses while avoiding obstacles
public class PathOptimizer {


    public static final double DEFAULT_MARGIN = 1;

    public static final double TARGET_CELL = 1;
    public static final int MAX_CELLS_PER_SIDE = 260;

    public static final int SPEED_BUCKETS = 64;

    public static final double MIN_SWEEP_STEP = 0.4;
    public static final int MIN_SWEEP_SAMPLES = 240;
    public static final int MAX_SWEEP_SAMPLES = 6000;
    public static final int REFINEMENTS = 16;
    public static final int MAX_WAYPOINTS = 40;
    public static final double MIN_WAYPOINT_GAP = 1.5;
    public static final double MIN_CORNER_GAP = 7;
    public static final double EVEN_SPACING = 12;
    public static final double PUSH_STEP = 2;

    private final RobotFrame robotFrame;
    private final MecanumProfile mecanumProfile;

    private double margin = DEFAULT_MARGIN;
    private HeadingOp headingOp;

    private double headingSwing = 1;

    public PathOptimizer(RobotFrame robotFrame) {
        this(robotFrame, null);
    }

    /// @param mecanumProfile lets the search cost routes by time rather than distance, may be null
    public PathOptimizer(RobotFrame robotFrame, MecanumProfile mecanumProfile) {

        this.robotFrame = robotFrame;
        this.mecanumProfile = mecanumProfile;
    }

    /// Extra slack kept between the robot and every obstacle, in inches.
    public PathOptimizer setMargin(double margin) {

        this.margin = Math.max(0, margin);
        return this;
    }

    /// Sets how heading will be handled on the paths produced (defaults to tangential heading).
    public PathOptimizer setHeadingOp(HeadingOp headingOp) {

        this.headingOp = headingOp;
        return this;
    }

    /// Sets a different heading control for each part of the paths produced.
    public PathOptimizer setHeadingOp(HeadingOp.Slice... slices) {

        this.headingOp = HeadingOp.compound(slices);
        return this;
    }

    /// @param ifOptimizationImpossibleMovement what to follow when no path can clear the obstacles
    public Movement produceEfficientPath(Pose startPose, Pose endPose, Movement ifOptimizationImpossibleMovement, Obstacle... obstacles) {

        Movement path = plan(startPose, endPose, obstacles);

        return path != null ? path : ifOptimizationImpossibleMovement;
    }

    /// When no path can clear the obstacles the robot holds where it is instead of driving.
    public Movement produceEfficientPath(Pose startPose, Pose endPose, Obstacle... obstacles) {

        Movement path = plan(startPose, endPose, obstacles);

        return path != null ? path : stay(startPose);
    }

    private Movement stay(Pose pose) {

        return new HermiteSpline(pose.copy(), pose.copy())
                .setHeadingOp(HeadingOp.constantHeading(pose.heading));
    }

    private HermiteSpline spline(Pose... waypoints) {

        HermiteSpline built = new HermiteSpline(waypoints);

        if (headingOp != null) return built.setHeadingOp(headingOp);

        double length = 0;

        for (int i = 1; i < waypoints.length; i++)
            length += Math.hypot(waypoints[i].x - waypoints[i - 1].x, waypoints[i].y - waypoints[i - 1].y);

        //pointed along the path is both the quickest way to cover ground and the narrowest way
        // through a gap, so the swings onto and off it are held to about a robot length, and
        // dropped altogether on the second pass if that is what it takes to fit through
        double swing = headingSwing * robotFrame.getRadius() * 2d / Math.max(length, 1e-9);

        return built.setHeadingOp(HeadingOp.efficientHeading(
                waypoints[0].heading, waypoints[waypoints.length - 1].heading, swing, swing));
    }

    private Movement plan(Pose startPose, Pose endPose, Obstacle[] obstacles) {

        if (obstacles == null || obstacles.length == 0) return spline(startPose.copy(), endPose.copy());

        //nothing can be routed out of or into a spot the robot already does not fit in
        if (hits(startPose.x, startPose.y, startPose.heading, obstacles)) return null;
        if (hits(endPose.x, endPose.y, endPose.heading, obstacles)) return null;

        HermiteSpline direct = spline(startPose.copy(), endPose.copy());
        if (firstHit(direct, obstacles, Math.hypot(endPose.x - startPose.x, endPose.y - startPose.y)) < 0) return direct;

        Field field = new Field(obstacles, startPose, endPose, robotFrame.getRadius() + margin);

        double outer = robotFrame.getRadius() + margin;
        double inner = Math.max(robotFrame.getInnerRadius(), 0.5) + margin;

        double[] radii = { outer, (outer + inner) / 2d, inner };

        for (double swing : new double[]{1, 0}) {

            headingSwing = swing;

            for (double radius : radii) {

                double[][] corners = field.route(radius, startPose, endPose, headingOp, mecanumProfile);
                if (corners == null) continue;

                HermiteSpline path = fit(corners, startPose, endPose, obstacles);
                if (path != null) return path;

                //a second try with the corners walked into open ground, which is room the curve needs
                path = fit(field.relax(corners, radius), startPose, endPose, obstacles);
                if (path != null) return path;
            }
        }

        return null;
    }

    private HermiteSpline fit(double[][] corners, Pose startPose, Pose endPose, Obstacle[] obstacles) {

        ArrayList<double[]> points = new ArrayList<>();
        for (double[] corner : corners) points.add(corner.clone());

        for (int attempt = 0; attempt <= REFINEMENTS; attempt++) {

            HermiteSpline path = build(points, startPose, endPose);

            double hit = firstHit(path, obstacles, walk(points));
            if (hit < 0) return simplify(points, startPose, endPose, obstacles);

            if (points.size() >= MAX_WAYPOINTS) return null;

            Pose strayed = path.sample(hit);

            if (attempt < REFINEMENTS / 2 && push(points, strayed.x, strayed.y, obstacles)) continue;

            if (!split(points, strayed.x, strayed.y)) return null;
        }

        return null;
    }

    private HermiteSpline simplify(ArrayList<double[]> points, Pose startPose, Pose endPose, Obstacle[] obstacles) {

        HermiteSpline best = build(points, startPose, endPose);

        for (boolean dropped = true; dropped && points.size() > 2; ) {

            dropped = false;

            for (int i = 1; i + 1 < points.size(); i++) {

                double[] removed = points.remove(i);

                HermiteSpline trial = build(points, startPose, endPose);

                if (firstHit(trial, obstacles, walk(points)) < 0) {
                    best = trial;
                    dropped = true;
                    break;
                }

                points.add(i, removed);
            }
        }

        //a short stretch beside a long one makes the curve overshoot, so the stretches are evened out
        double[][] traced = trace(best);

        for (double spacing : new double[]{EVEN_SPACING, EVEN_SPACING * 1.75, EVEN_SPACING * 3}) {

            ArrayList<double[]> even = resample(traced, spacing);
            HermiteSpline smooth = build(even, startPose, endPose);

            if (firstHit(smooth, obstacles, walk(even)) < 0) return smooth;
        }

        return best;
    }

    private double[][] trace(HermiteSpline path) {

        final int fine = 400;

        double[] x = new double[fine + 1], y = new double[fine + 1], along = new double[fine + 1];

        for (int i = 0; i <= fine; i++) {

            Pose point = path.sample((double) i / fine);

            x[i] = point.x;
            y[i] = point.y;

            if (i > 0) along[i] = along[i - 1] + Math.hypot(x[i] - x[i - 1], y[i] - y[i - 1]);
        }

        return new double[][]{x, y, along};
    }

    private ArrayList<double[]> resample(double[][] traced, double wanted) {

        //points follow the curve evenly preventing its shape from making loops

        final double[] x = traced[0];
        final double[] y = traced[1];
        final double[] along = traced[2];

        final int fine = along.length - 1;

        double spacing = Math.max(wanted, along[fine] / (MAX_WAYPOINTS - 1));
        int pieces = Math.max(1, (int) Math.round(along[fine] / spacing));

        ArrayList<double[]> points = new ArrayList<>();

        for (int k = 0, i = 0; k <= pieces; k++) {

            double reached = along[fine] * k / pieces;

            while (i + 2 <= fine && along[i + 1] < reached) i++;

            double length = along[i + 1] - along[i];
            double t = length > 0 ? (reached - along[i]) / length : 0;

            points.add(new double[]{x[i] + (x[i + 1] - x[i]) * t, y[i] + (y[i + 1] - y[i]) * t});
        }

        return points;
    }

    private HermiteSpline build(ArrayList<double[]> points, Pose startPose, Pose endPose) {

        Pose[] waypoints = new Pose[points.size()];

        for (int i = 0; i < points.size(); i++) {

            double[] p = points.get(i);
            waypoints[i] = new Pose(p[0], p[1], i == 0 ? startPose.heading : endPose.heading);
        }

        return spline(waypoints);
    }

    private double walk(ArrayList<double[]> points) {

        double length = 0;

        for (int i = 0; i + 1 < points.size(); i++) {
            length += Math.hypot(points.get(i + 1)[0] - points.get(i)[0], points.get(i + 1)[1] - points.get(i)[1]);
        }

        return length;
    }

    private boolean push(ArrayList<double[]> points, double px, double py, Obstacle[] obstacles) {

        if (points.size() < 3) return false;

        int nearest = -1;
        double nearestDistance = Double.MAX_VALUE;

        for (int i = 1; i + 1 < points.size(); i++) {

            double distance = Math.hypot(points.get(i)[0] - px, points.get(i)[1] - py);

            if (distance < nearestDistance) {
                nearestDistance = distance;
                nearest = i;
            }
        }

        if (nearest < 0) return false;

        double step = 0.5;
        double awayX = (room(px + step, py, obstacles) - room(px - step, py, obstacles)) / (2 * step);
        double awayY = (room(px, py + step, obstacles) - room(px, py - step, obstacles)) / (2 * step);

        double length = Math.hypot(awayX, awayY);
        if (length < 1e-9) return false;

        points.get(nearest)[0] += awayX / length * PUSH_STEP;
        points.get(nearest)[1] += awayY / length * PUSH_STEP;

        return true;
    }

    private double room(double x, double y, Obstacle[] obstacles) {

        double best = Double.MAX_VALUE;

        for (Obstacle obstacle : obstacles) best = Math.min(best, obstacle.clearance(x, y));

        return best;
    }

    private boolean split(ArrayList<double[]> points, double px, double py) {

        //makes the closest stretch shorter by pulling the curve toward the corners

        int nearest = -1;
        double nearestDistance = Double.MAX_VALUE;

        for (int i = 0; i + 1 < points.size(); i++) {

            double[] a = points.get(i), b = points.get(i + 1);
            double distance = Outline.pointToSegment(px, py, a[0], a[1], b[0], b[1]);

            if (distance < nearestDistance) {
                nearestDistance = distance;
                nearest = i;
            }
        }

        if (nearest < 0) return false;

        for (int reach = 0; reach < points.size(); reach++) {

            for (int side = 0; side < 2; side++) {

                int i = side == 0 ? nearest - reach : nearest + reach;
                if (i < 0 || i + 1 >= points.size()) continue;

                double[] a = points.get(i), b = points.get(i + 1);
                if (Math.hypot(b[0] - a[0], b[1] - a[1]) < 2 * MIN_WAYPOINT_GAP) continue;

                points.add(i + 1, new double[]{(a[0] + b[0]) / 2d, (a[1] + b[1]) / 2d});

                return true;
            }
        }

        return false;
    }

    /// @return the first spot along the curve where the robot would not fit, or -1 when it all clears
    private double firstHit(HermiteSpline path, Obstacle[] obstacles, double length) {

        double step = Math.max(MIN_SWEEP_STEP, margin);
        int steps = (int) MathHelper.clamp(length / step, MIN_SWEEP_SAMPLES, MAX_SWEEP_SAMPLES);

        Pose previous = path.sample(0);
        if (hits(previous.x, previous.y, previous.heading, obstacles)) return 0;

        for (int i = 1; i <= steps; i++) {

            double t = (double) i / steps;
            Pose pose = path.sample(t);

            //turning swings the corners further than the center travels so both are counted
            double swept = Math.hypot(pose.x - previous.x, pose.y - previous.y)
                    + robotFrame.getRadius() * Math.abs(MathHelper.normalizeAngleRad(pose.heading - previous.heading));

            int splits = (int) Math.min(64, Math.ceil(swept / step));

            for (int k = 1; k < splits; k++) {

                double between = ((double) (i - 1) + (double) k / splits) / steps;
                Pose inner = path.sample(between);

                if (hits(inner.x, inner.y, inner.heading, obstacles)) return between;
            }

            if (hits(pose.x, pose.y, pose.heading, obstacles)) return t;

            previous = pose;
        }

        return -1;
    }

    private final double[] bodyX = new double[64], bodyY = new double[64];

    private boolean hits(double x, double y, double heading, Obstacle[] obstacles) {

        int count = robotFrame.getPointCount();
        robotFrame.writeGlobal(x, y, heading, bodyX, bodyY);

        double minX = Double.MAX_VALUE, minY = Double.MAX_VALUE;
        double maxX = -Double.MAX_VALUE, maxY = -Double.MAX_VALUE;

        for (int i = 0; i < count; i++) {
            minX = Math.min(minX, bodyX[i]); maxX = Math.max(maxX, bodyX[i]);
            minY = Math.min(minY, bodyY[i]); maxY = Math.max(maxY, bodyY[i]);
        }

        for (Obstacle obstacle : obstacles) {

            Outline outline = obstacle.getOutline();

            //the body cannot reach past its own radius, so most samples stop at one distance
            double reach = outline.boundRadius + robotFrame.getRadius() + margin;
            double awayX = x - outline.centerX, awayY = y - outline.centerY;

            if (awayX * awayX + awayY * awayY > reach * reach) continue;

            if (outline.clearance(x, y) > robotFrame.getRadius() + margin) continue;

            if (Outline.overlap(outline, bodyX, bodyY, count, minX, minY, maxX, maxY)) return true;

            if (margin > 0 && gap(outline, count, minX, minY, maxX, maxY) < margin) return true;
        }

        return false;
    }

    //closest approach between the robot's outline and an obstacle's, only reached when they miss
    private double gap(Outline outline, int count, double minX, double minY, double maxX, double maxY) {

        double best = Double.MAX_VALUE;

        //a corner further than the margin from the other shape's box cannot be the closest one
        for (int i = 0; i < outline.count; i++) {

            if (outline.x[i] < minX - margin || outline.x[i] > maxX + margin || outline.y[i] < minY - margin || outline.y[i] > maxY + margin) {
                continue;
            }

            for (int k = 0, l = count - 1; k < count; l = k++) {
                best = Math.min(best, Outline.pointToSegment(outline.x[i], outline.y[i], bodyX[l], bodyY[l], bodyX[k], bodyY[k]));
            }
        }

        for (int i = 0; i < count; i++) {

            if (bodyX[i] < outline.minX - margin || bodyX[i] > outline.maxX + margin || bodyY[i] < outline.minY - margin || bodyY[i] > outline.maxY + margin) {
                continue;
            }

            best = Math.min(best, outline.distanceToEdge(bodyX[i], bodyY[i]));
        }

        return best;
    }

}

package org.firstinspires.ftc.teamcode.path.optimization;

import org.apache.commons.math3.util.FastMath;

import org.firstinspires.ftc.teamcode.following.chassis.MecanumProfile;
import org.firstinspires.ftc.teamcode.path.HeadingOp;
import org.firstinspires.ftc.teamcode.path.HermiteSpline;
import org.firstinspires.ftc.teamcode.path.Movement;
import org.firstinspires.ftc.teamcode.util.MathHelper;
import org.firstinspires.ftc.teamcode.util.MiniHeap;
import org.firstinspires.ftc.teamcode.util.Pose;

import java.util.ArrayList;
import java.util.Arrays;

/// Builds the quickest path between two poses that the robot's body can actually fit through.
public class PathOptimizer {

    private static final double DEFAULT_MARGIN = 1;

    private static final double TARGET_CELL = 1;
    private static final int MAX_CELLS_PER_SIDE = 260;

    private static final int SPEED_BUCKETS = 64;

    //the body never moves further than the margin between two checks, so nothing can slip through one
    private static final double MIN_SWEEP_STEP = 0.4;
    private static final int MIN_SWEEP_SAMPLES = 240;
    private static final int MAX_SWEEP_SAMPLES = 6000;
    private static final int REFINEMENTS = 16;
    private static final int MAX_WAYPOINTS = 40;
    private static final double MIN_WAYPOINT_GAP = 1.5;
    private static final double MIN_CORNER_GAP = 7;
    private static final double EVEN_SPACING = 12;
    private static final double PUSH_STEP = 2;

    private final RobotFrame robotFrame;
    private final MecanumProfile mecanumProfile;

    private double margin = DEFAULT_MARGIN;
    private HeadingOp headingOp;

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

        return headingOp == null ? built : built.setHeadingOp(headingOp);
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

        //room to spare first, since a route with space is smoother and quicker to drive than a
        // shorter one that hugs everything, then tighter tries only if nothing else gets through
        double[] radii = { outer, (outer + inner) / 2d, inner };

        for (double radius : radii) {

            double[][] corners = field.route(radius, startPose, endPose, headingOp, mecanumProfile);
            if (corners == null) continue;

            HermiteSpline path = fit(corners, startPose, endPose, obstacles);
            if (path != null) return path;

            //a second try with the corners walked into open ground, which is room the curve needs
            path = fit(field.relax(corners, radius), startPose, endPose, obstacles);
            if (path != null) return path;
        }

        return null;
    }

    //grows the corner list until the spline itself, not just the corners, clears everything
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

    //the fewest corners that still clear everything, which is also the smoothest way round
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
        for (double spacing : new double[]{EVEN_SPACING, EVEN_SPACING * 1.75, EVEN_SPACING * 3}) {

            ArrayList<double[]> even = spread(points, spacing);
            HermiteSpline smooth = build(even, startPose, endPose);

            if (firstHit(smooth, obstacles, walk(even)) < 0) return smooth;
        }

        return best;
    }

    private ArrayList<double[]> spread(ArrayList<double[]> corners, double wanted) {

        ArrayList<double[]> points = new ArrayList<>();

        double spacing = Math.max(wanted, walk(corners) / (MAX_WAYPOINTS - 1));

        for (int i = 0; i + 1 < corners.size(); i++) {

            double[] a = corners.get(i), b = corners.get(i + 1);
            int pieces = Math.max(1, (int) Math.round(Math.hypot(b[0] - a[0], b[1] - a[1]) / spacing));

            for (int k = 0; k < pieces; k++) {

                double t = (double) k / pieces;
                points.add(new double[]{a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t});
            }
        }

        points.add(corners.get(corners.size() - 1).clone());

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

    //moves the corner out a little so the curve can cut the turn
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

    //makes the closest stretch shorter, pulling the curve back toward the corners
    private boolean split(ArrayList<double[]> points, double px, double py) {

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

        //the stray may be beside a corner rather than a stretch, so its neighbours are fair game too
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

            //turning swings the corners further than the centre travels, so both are counted
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

    /// The field turned into a clearance map, so a route can be searched without touching outlines again.
    private static final class Field {

        final double originX, originY, cell;
        final int nx, ny;
        final double[] clearance;

        Field(Obstacle[] obstacles, Pose startPose, Pose endPose, double reach) {

            double minX = Math.min(startPose.x, endPose.x), maxX = Math.max(startPose.x, endPose.x);
            double minY = Math.min(startPose.y, endPose.y), maxY = Math.max(startPose.y, endPose.y);

            for (Obstacle obstacle : obstacles) {
                minX = Math.min(minX, obstacle.getMinX()); maxX = Math.max(maxX, obstacle.getMaxX());
                minY = Math.min(minY, obstacle.getMinY()); maxY = Math.max(maxY, obstacle.getMaxY());
            }

            double pad = reach + 4;
            minX -= pad; minY -= pad; maxX += pad; maxY += pad;

            double width = maxX - minX, height = maxY - minY;
            double size = Math.max(TARGET_CELL, Math.max(width, height) / MAX_CELLS_PER_SIDE);

            cell = size;
            originX = minX;
            originY = minY;
            nx = Math.max(2, (int) Math.ceil(width / size) + 1);
            ny = Math.max(2, (int) Math.ceil(height / size) + 1);

            double[] occupied = new double[nx * ny];
            Arrays.fill(occupied, Double.MAX_VALUE);

            double touch = cell / 2d;

            for (Obstacle obstacle : obstacles) {

                Outline outline = obstacle.getOutline();

                int lowX = Math.max(0, (int) Math.floor((outline.minX - touch - originX) / cell));
                int highX = Math.min(nx - 1, (int) Math.ceil((outline.maxX + touch - originX) / cell));
                int lowY = Math.max(0, (int) Math.floor((outline.minY - touch - originY) / cell));
                int highY = Math.min(ny - 1, (int) Math.ceil((outline.maxY + touch - originY) / cell));

                for (int iy = lowY; iy <= highY; iy++) {

                    double wy = originY + iy * cell;

                    for (int ix = lowX; ix <= highX; ix++) {

                        double wx = originX + ix * cell;

                        if (outline.clearance(wx, wy) <= touch) occupied[iy * nx + ix] = 0;
                    }
                }
            }

            clearance = distanceTransform(occupied, nx, ny);

            //cells are marked from their middles
            double bias = 0.35 * cell;

            for (int i = 0; i < clearance.length; i++) {
                clearance[i] = Math.max(0, Math.sqrt(clearance[i]) * cell - bias + (cell / 2d));
            }
        }

        double clearanceAt(double wx, double wy) {

            int ix = (int) Math.round((wx - originX) / cell);
            int iy = (int) Math.round((wy - originY) / cell);

            if (ix < 0 || iy < 0 || ix >= nx || iy >= ny) return 0;

            return clearance[iy * nx + ix];
        }

        boolean lineClear(double ax, double ay, double bx, double by, double radius) {

            double length = Math.hypot(bx - ax, by - ay);
            int steps = Math.max(1, (int) Math.ceil(length / (cell * 0.5)));

            for (int i = 0; i <= steps; i++) {

                double t = (double) i / steps;
                if (clearanceAt(ax + (bx - ax) * t, ay + (by - ay) * t) < radius) return false;
            }

            return true;
        }

        /// @return the corners of a route, or null when the robot cannot get there at this radius
        double[][] route(double radius, Pose startPose, Pose endPose, HeadingOp headingOp, MecanumProfile profile) {

            int startCell = nearestFree(startPose.x, startPose.y, radius);
            int endCell = nearestFree(endPose.x, endPose.y, radius);

            if (startCell < 0 || endCell < 0) return null;

            int[] cameFrom = search(startCell, endCell, radius, headingOp, profile, endPose);
            if (cameFrom == null) return null;

            ArrayList<double[]> raw = new ArrayList<>();

            for (int node = endCell; node != -1; node = cameFrom[node])
                raw.add(0, new double[]{originX + (node % nx) * cell, originY + (node / nx) * cell});

            raw.set(0, new double[]{startPose.x, startPose.y});
            raw.set(raw.size() - 1, new double[]{endPose.x, endPose.y});

            return shortcut(raw, radius);
        }

        private int nearestFree(double wx, double wy, double radius) {

            int ix = (int) Math.round((wx - originX) / cell);
            int iy = (int) Math.round((wy - originY) / cell);

            for (int ring = 0; ring <= 6; ring++) {

                for (int dy = -ring; dy <= ring; dy++) {

                    for (int dx = -ring; dx <= ring; dx++) {

                        if (Math.max(Math.abs(dx), Math.abs(dy)) != ring) continue;

                        int cx = ix + dx, cy = iy + dy;
                        if (cx < 0 || cy < 0 || cx >= nx || cy >= ny) continue;

                        if (clearance[cy * nx + cx] >= radius) return cy * nx + cx;
                    }
                }
            }

            return -1;
        }

        private int[] search(int start, int goal, double radius, HeadingOp headingOp, MecanumProfile profile, Pose endPose) {

            double[] speeds = speedTable(profile);
            double bestSpeed = 0;
            for (double speed : speeds) bestSpeed = Math.max(bestSpeed, speed);

            int size = nx * ny;

            double[] cost = new double[size];
            double[] travelled = new double[size];
            int[] cameFrom = new int[size];
            boolean[] done = new boolean[size];

            Arrays.fill(cost, Double.MAX_VALUE);
            Arrays.fill(cameFrom, -1);

            MiniHeap heap = new MiniHeap(size);

            cost[start] = 0;
            travelled[start] = 0;
            heap.push(start, 0);

            double goalX = originX + (goal % nx) * cell, goalY = originY + (goal / nx) * cell;

            while (!heap.isEmpty()) {

                int node = heap.pop();
                if (done[node]) continue;
                done[node] = true;

                if (node == goal) return cameFrom;

                int ix = node % nx, iy = node / nx;
                double wx = originX + ix * cell, wy = originY + iy * cell;

                for (int dy = -1; dy <= 1; dy++) {

                    for (int dx = -1; dx <= 1; dx++) {

                        if (dx == 0 && dy == 0) continue;

                        int cx = ix + dx, cy = iy + dy;
                        if (cx < 0 || cy < 0 || cx >= nx || cy >= ny) continue;

                        int next = cy * nx + cx;
                        if (done[next] || clearance[next] < radius) continue;

                        //a diagonal step may not cut a corner the robot would clip
                        if (dx != 0 && dy != 0
                                && (clearance[iy * nx + cx] < radius || clearance[cy * nx + ix] < radius)) continue;

                        double step = (dx != 0 && dy != 0) ? cell * Math.sqrt(2) : cell;
                        double nextTravelled = travelled[node] + step;

                        double speed = bestSpeed;

                        if (speeds.length > 1) {

                            double toGoal = Math.hypot(goalX - wx, goalY - wy);
                            double progress = nextTravelled / Math.max(1e-9, nextTravelled + toGoal);
                            double travelAngle = FastMath.atan2(dy, dx);
                            double bodyAngle = headingOp == null
                                    ? travelAngle
                                    : headingOp.heading(progress, wx, wy, travelAngle, false);

                            speed = lookUp(speeds, travelAngle - bodyAngle);
                        }

                        double candidate = cost[node] + step / speed;
                        if (candidate >= cost[next]) continue;

                        cost[next] = candidate;
                        travelled[next] = nextTravelled;
                        cameFrom[next] = node;

                        double remaining = Math.hypot(goalX - (originX + cx * cell), goalY - (originY + cy * cell));
                        heap.push(next, candidate + remaining / bestSpeed);
                    }
                }
            }

            return null;
        }

        private double[] speedTable(MecanumProfile profile) {

            if (profile == null) return new double[]{1};

            double[] table = new double[SPEED_BUCKETS + 1];

            for (int i = 0; i <= SPEED_BUCKETS; i++) {
                table[i] = profile.getMaxVelocity(Math.PI / 2d * i / SPEED_BUCKETS);
            }

            return table;
        }

        private static double lookUp(double[] table, double angle) {

            double folded = Math.abs(MathHelper.normalizeAngleRad(angle));
            if (folded > Math.PI / 2d) folded = Math.PI - folded;

            int index = (int) Math.round(folded / (Math.PI / 2d) * SPEED_BUCKETS);

            return table[MathHelper.clamp(index, 0, SPEED_BUCKETS)];
        }

        /// Moves each corner toward more open space, giving the curve more room than a corner needs.
        double[][] relax(double[][] corners, double radius) {

            double[][] moved = new double[corners.length][];
            for (int i = 0; i < corners.length; i++) moved[i] = corners[i].clone();

            for (int round = 0; round < 2; round++) {

                for (int i = 1; i + 1 < moved.length; i++) {

                    double bestX = moved[i][0], bestY = moved[i][1];
                    double best = clearanceAt(bestX, bestY);

                    for (int step = 0; step < 8; step++) {

                        double angle = 2d * Math.PI * step / 8;

                        for (double reach = cell; reach <= 2 * cell; reach += cell) {

                            double tryX = moved[i][0] + reach * FastMath.cos(angle);
                            double tryY = moved[i][1] + reach * FastMath.sin(angle);

                            if (clearanceAt(tryX, tryY) <= best) continue;

                            if (!lineClear(moved[i - 1][0], moved[i - 1][1], tryX, tryY, radius)) continue;
                            if (!lineClear(tryX, tryY, moved[i + 1][0], moved[i + 1][1], radius)) continue;

                            best = clearanceAt(tryX, tryY);
                            bestX = tryX;
                            bestY = tryY;
                        }
                    }

                    moved[i][0] = bestX;
                    moved[i][1] = bestY;
                }
            }

            return moved;
        }

        // Keeps only the corners that are needed to connect the straight parts.
        private double[][] shortcut(ArrayList<double[]> raw, double radius) {

            ArrayList<double[]> kept = new ArrayList<>();
            kept.add(raw.get(0));

            int i = 0;

            while (i < raw.size() - 1) {

                int j = raw.size() - 1;

                while (j > i + 1 && !lineClear(raw.get(i)[0], raw.get(i)[1], raw.get(j)[0], raw.get(j)[1], radius)) j--;

                kept.add(raw.get(j));
                i = j;
            }

            //corners a few inches apart only make the curve whip round, so they are merged
            for (int k = kept.size() - 2; k > 0; k--) {

                double[] here = kept.get(k), after = kept.get(k + 1);

                if (Math.hypot(after[0] - here[0], after[1] - here[1]) < MIN_CORNER_GAP && lineClear(kept.get(k - 1)[0], kept.get(k - 1)[1], after[0], after[1], radius)) {
                    kept.remove(k);
                }
            }

            return kept.toArray(new double[0][]);
        }

        //genuine W Felzenszwalb bro
        private static double[] distanceTransform(double[] source, int nx, int ny) {

            double[] result = source.clone();

            final int longest = Math.max(nx, ny);

            double[] line = new double[longest];
            double[] out = new double[longest];
            int[] hull = new int[longest];
            double[] edge = new double[longest + 1];

            for (int x = 0; x < nx; x++) {

                for (int y = 0; y < ny; y++) {
                    line[y] = result[y * nx + x];
                }
                transform(line, out, hull, edge, ny);
                for (int y = 0; y < ny; y++) {
                    result[y * nx + x] = out[y];
                }
            }

            for (int y = 0; y < ny; y++) {

                for (int x = 0; x < nx; x++) {
                    line[x] = result[y * nx + x];
                }

                transform(line, out, hull, edge, nx);

                for (int x = 0; x < nx; x++) {
                    result[y * nx + x] = out[x];
                }
            }

            return result;
        }

        private static void transform(double[] f, double[] d, int[] v, double[] z, int n) {

            int k = 0;

            v[0] = 0;
            z[0] = -Double.MAX_VALUE;
            z[1] = Double.MAX_VALUE;

            for (int q = 1; q < n; q++) {

                double s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2d * q - 2d * v[k]);

                while (s <= z[k]) {

                    k--;
                    s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2d * q - 2d * v[k]);
                }

                k++;
                v[k] = q;
                z[k] = s;
                z[k + 1] = Double.MAX_VALUE;
            }

            k = 0;

            for (int q = 0; q < n; q++) {

                while (z[k + 1] < q) k++;

                double gap = q - v[k];
                d[q] = gap * gap + f[v[k]];
            }
        }
    }
}

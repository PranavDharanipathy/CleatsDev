package org.firstinspires.ftc.teamcode.path.optimization;

import static org.firstinspires.ftc.teamcode.path.optimization.PathOptimizer.MAX_CELLS_PER_SIDE;
import static org.firstinspires.ftc.teamcode.path.optimization.PathOptimizer.MIN_CORNER_GAP;
import static org.firstinspires.ftc.teamcode.path.optimization.PathOptimizer.SPEED_BUCKETS;
import static org.firstinspires.ftc.teamcode.path.optimization.PathOptimizer.TARGET_CELL;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.teamcode.following.chassis.MecanumProfile;
import org.firstinspires.ftc.teamcode.path.HeadingOp;
import org.firstinspires.ftc.teamcode.util.MathHelper;
import org.firstinspires.ftc.teamcode.util.MiniHeap;
import org.firstinspires.ftc.teamcode.util.Pose;

import java.util.ArrayList;
import java.util.Arrays;

public final class Field {

    final double originX, originY, cell;
    final int nx, ny;
    final double[] clearance;

    public Field(Obstacle[] obstacles, Pose startPose, Pose endPose, double reach) {

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
    public double[][] route(double radius, Pose startPose, Pose endPose, HeadingOp headingOp, MecanumProfile profile) {

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

        double goalX = originX + (goal % nx) * cell, goalY = originY + ((double) goal / nx) * cell;

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

    public double[][] relax(double[][] corners, double radius) {

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

    private double[][] shortcut(ArrayList<double[]> raw, double radius) {

        //keeps only the corners that are needed to connect the straight parts

        ArrayList<double[]> kept = new ArrayList<>();
        kept.add(raw.get(0));

        int i = 0;

        while (i < raw.size() - 1) {

            int j = raw.size() - 1;

            while (j > i + 1 && !lineClear(raw.get(i)[0], raw.get(i)[1], raw.get(j)[0], raw.get(j)[1], radius)) j--;

            kept.add(raw.get(j));
            i = j;
        }

        //if corners have too little gap between them, they're merged
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

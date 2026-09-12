package org.firstinspires.ftc.teamcode.path.optimization;

import org.apache.commons.math3.util.FastMath;

public class Outline {

    final double[] x, y;
    final int count;

    final double minX, minY, maxX, maxY;

    final double centerX, centerY, boundRadius;

    public Outline(CollisionPoint... points) {

        CollisionPoint[] wound = wind(points);

        count = wound.length;
        x = new double[count];
        y = new double[count];

        for (int i = 0; i < count; i++) {
            x[i] = wound[i].x;
            y[i] = wound[i].y;
        }

        if (signedArea() < 0) reverse();

        double lowX = Double.MAX_VALUE, lowY = Double.MAX_VALUE;
        double highX = -Double.MAX_VALUE, highY = -Double.MAX_VALUE;

        for (int i = 0; i < count; i++) {
            lowX = Math.min(lowX, x[i]);
            lowY = Math.min(lowY, y[i]);
            highX = Math.max(highX, x[i]);
            highY = Math.max(highY, y[i]);
        }

        minX = lowX; minY = lowY; maxX = highX; maxY = highY;

        centerX = (minX + maxX) / 2d;
        centerY = (minY + maxY) / 2d;
        boundRadius = radiusFrom(centerX, centerY);
    }

    private static CollisionPoint[] wind(CollisionPoint[] points) {

        if (isSimple(points)) return points.clone();

        double centerX = 0, centerY = 0;

        for (CollisionPoint point : points) {
            centerX += point.x;
            centerY += point.y;
        }

        final double middleX = centerX / points.length;
        final double middleY = centerY / points.length;

        CollisionPoint[] sorted = points.clone();

        java.util.Arrays.sort(sorted, (a, b) -> Double.compare(
                FastMath.atan2(a.y - middleY, a.x - middleX),
                FastMath.atan2(b.y - middleY, b.x - middleX)
        ));

        if (isSimple(sorted)) return sorted;

        throw new IllegalArgumentException("These points cross over themselves, they don't make one outline!");
    }

    static boolean isSimple(CollisionPoint[] p) {

        int n = p.length;

        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {

                if (j == i + 1 || (i == 0 && j == n - 1)) continue; //edges that share a corner

                if (segmentsCross(p[i].x, p[i].y, p[(i + 1) % n].x, p[(i + 1) % n].y,
                        p[j].x, p[j].y, p[(j + 1) % n].x, p[(j + 1) % n].y)) return false;
            }
        }

        return true;
    }

    private double signedArea() {

        double sum = 0;

        for (int i = 0, j = count - 1; i < count; j = i++) sum += x[j] * y[i] - x[i] * y[j];

        return sum / 2d;
    }

    private void reverse() {

        for (int i = 0, j = count - 1; i < j; i++, j--) {

            double tx = x[i]; x[i] = x[j]; x[j] = tx;
            double ty = y[i]; y[i] = y[j]; y[j] = ty;
        }
    }

    public boolean contains(double px, double py) {

        if (px < minX || px > maxX || py < minY || py > maxY) return false;

        boolean inside = false;

        for (int i = 0, j = count - 1; i < count; j = i++) {

            if ((y[i] > py) != (y[j] > py)
                    && px < (x[j] - x[i]) * (py - y[i]) / (y[j] - y[i]) + x[i]) inside = !inside;
        }

        return inside;
    }

    /// Distance from a point to the outline itself, whichever side the point is on.
    public double distanceToEdge(double px, double py) {

        double best = Double.MAX_VALUE;

        for (int i = 0, j = count - 1; i < count; j = i++)
            best = Math.min(best, pointToSegment(px, py, x[j], y[j], x[i], y[i]));

        return best;
    }

    /// @return 0 when the point is inside, otherwise how far outside it sits
    public double clearance(double px, double py) {
        return contains(px, py) ? 0 : distanceToEdge(px, py);
    }

    public double radiusFrom(double px, double py) {

        double best = 0;

        for (int i = 0; i < count; i++) best = Math.max(best, Math.hypot(x[i] - px, y[i] - py));

        return best;
    }

    public static double pointToSegment(double px, double py, double ax, double ay, double bx, double by) {

        double dx = bx - ax, dy = by - ay;
        double lengthSquared = dx * dx + dy * dy;

        if (lengthSquared <= 0) return Math.hypot(px - ax, py - ay);

        double t = ((px - ax) * dx + (py - ay) * dy) / lengthSquared;
        t = t < 0 ? 0 : (t > 1 ? 1 : t);

        return Math.hypot(px - (ax + t * dx), py - (ay + t * dy));
    }

    public static boolean segmentsCross(double ax, double ay, double bx, double by, double cx, double cy, double dx, double dy) {

        double d1 = side(cx, cy, dx, dy, ax, ay);
        double d2 = side(cx, cy, dx, dy, bx, by);
        double d3 = side(ax, ay, bx, by, cx, cy);
        double d4 = side(ax, ay, bx, by, dx, dy);

        if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) return true;

        return (d1 == 0 && onSegment(cx, cy, dx, dy, ax, ay))
                || (d2 == 0 && onSegment(cx, cy, dx, dy, bx, by))
                || (d3 == 0 && onSegment(ax, ay, bx, by, cx, cy))
                || (d4 == 0 && onSegment(ax, ay, bx, by, dx, dy));
    }

    private static double side(double ax, double ay, double bx, double by, double px, double py) {
        return (bx - ax) * (py - ay) - (by - ay) * (px - ax);
    }

    private static boolean onSegment(double ax, double ay, double bx, double by, double px, double py) {
        return Math.min(ax, bx) <= px && px <= Math.max(ax, bx) && Math.min(ay, by) <= py && py <= Math.max(ay, by);
    }

    public int getPointCount() {
        return count;
    }

    public CollisionPoint[] getPoints() {

        CollisionPoint[] corners = new CollisionPoint[count];

        for (int i = 0; i < count; i++) {
            corners[i] = new CollisionPoint(x[i], y[i]);
        }

        return corners;
    }

    public double getMinX() {
        return minX;
    }

    public double getMinY() {
        return minY;
    }

    public double getMaxX() {
        return maxX;
    }

    public double getMaxY() {
        return maxY;
    }

    /// @return how far the furthest corner sits from the middle of the outline's box
    public double getBoundRadius() {
        return boundRadius;
    }

    /// Whether two outlines share any area
    static boolean overlap(Outline a, double[] bx, double[] by, int bCount, double bMinX, double bMinY, double bMaxX, double bMaxY) {

        if (a.maxX < bMinX || a.minX > bMaxX || a.maxY < bMinY || a.minY > bMaxY) return false;

        for (int i = 0; i < bCount; i++) if (a.contains(bx[i], by[i])) return true;

        for (int i = 0; i < a.count; i++) if (containsIn(bx, by, bCount, a.x[i], a.y[i])) return true;

        for (int i = 0, j = a.count - 1; i < a.count; j = i++)
            for (int k = 0, l = bCount - 1; k < bCount; l = k++)
                if (segmentsCross(a.x[j], a.y[j], a.x[i], a.y[i], bx[l], by[l], bx[k], by[k])) return true;

        return false;
    }

    private static boolean containsIn(double[] px, double[] py, int n, double qx, double qy) {

        boolean inside = false;

        for (int i = 0, j = n - 1; i < n; j = i++) {

            if ((py[i] > qy) != (py[j] > qy)
                    && qx < (px[j] - px[i]) * (qy - py[i]) / (py[j] - py[i]) + px[i]) inside = !inside;
        }

        return inside;
    }
}

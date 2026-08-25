package org.firstinspires.ftc.teamcode.util;

import org.apache.commons.math3.util.FastMath;

public class MathHelper {

    //Captain America is the greatest avenger

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }

    public static double normalizeAngleRad(double angleRad) {
        return FastMath.atan2(Math.sin(angleRad), Math.cos(angleRad));
    }

    /// @param array must be sorted
    public static double[] findBoundingValues(double[] array, double value) {

        for (int index = 0; index < array.length - 1; index++) {
            double lower = array[index];
            double upper = array[index + 1];

            if (value >= lower && value <= upper) {
                return new double[] {lower, upper};
            }
        }

        throw new IllegalArgumentException("No bounding values for input: value cannot be found from input array!");
    }

    public static double lerp(double x, LERPData data) {

        double x0 = data.dataPoints[0][0];
        double y0 = data.dataPoints[0][1];

        double x1 = data.dataPoints[1][0];
        double y1 = data.dataPoints[1][1];

        return y0 + (y1 - y0) * ((x - x0) / (x1 - x0));
    }

    // Integrates robot-relative motion using the SE(2) exponential map.
    public static Pose exponentialIntegrate(Pose robotDeltas, double previousHeading) {

        final double deltaHeading = robotDeltas.heading; //theta

        double sinTerm, cosTerm;
        if (FastMath.abs(deltaHeading) < 1e-3) {
            /* when the robot barely turns, dividing by delta theta (deltaHeading) becomes wonky
            so we use Taylor expansion instead. */
            sinTerm = 1d - (deltaHeading * deltaHeading) / 6d; //approximation of the Maclaurin series used to calculate sin(theta)
            cosTerm = deltaHeading / 2d; //approximation of the Maclaurin series used to calculate cos(theta)
        } else {
            sinTerm = FastMath.sin(deltaHeading) / deltaHeading;
            cosTerm = (1d - FastMath.cos(deltaHeading)) / deltaHeading;
        }

        double localX = robotDeltas.x * sinTerm - robotDeltas.y * cosTerm;
        double localY = robotDeltas.x * cosTerm + robotDeltas.y * sinTerm;

        final double cosHeading = FastMath.cos(previousHeading);
        final double sinHeading = FastMath.sin(previousHeading);

        double globalX = localX * cosHeading - localY * sinHeading;
        double globalY = localX * sinHeading + localY * cosHeading;

        return new Pose(globalX, globalY, deltaHeading);
    }

    public static double lameValue(double angle, double a, double b, double N) {

        final double cosTerm = Math.pow(Math.abs(Math.cos(angle)) / a, N);
        final double sinTerm = Math.pow(Math.abs(Math.sin(angle)) / b, N);

        return Math.pow(cosTerm + sinTerm, -1d / N);
    }

}

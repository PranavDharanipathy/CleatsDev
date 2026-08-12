package org.firstinspires.ftc.teamcode.following.chassis;

import org.firstinspires.ftc.teamcode.util.MathHelper;

/**
 * In practice, a mecanum chassis's top speed isn't the same in every direction since
 * due to slip and other factors, because of this the chassis's maximum speed is modeled
 * as a function of its desired angle of movement with respect to test-derived kinematic
 * terms.
 */
public class MecanumProfile {

    /*
    * === Profile shape ===
    *
    *        /\
    *       *  *
    *      *    *
    *    *        *
    * <              >
    *    *        *
    *      *    *
    *       *  *
    *        \/
    *
    *   - Lamé curve (superellipse)
     */

    private final double vmaxF, vmaxS, velocityShape;
    private final double amaxF, amaxS, accelerationShape;
    private final double dmaxF, dmaxS, decelerationShape;

    /// @param vmaxF is the top forward speed in inches per second
    /// @param vmaxS is the top strafe speed in inches per second
    /// @param vmaxD is the diagonal top speed in inches per second
    /// @param amaxF is the peak forward acceleration in inches per second squared
    /// @param amaxS is the peak strafe acceleration in inches per second squared
    /// @param amaxD is the diagonal peak acceleration in inches per second squared
    /// @param dmaxF is the peak forward deceleration in inches per second squared
    /// @param dmaxS is the peak strafe deceleration in inches per second squared
    /// @param dmaxD is the diagonal peak deceleration in inches per second squared
    public MecanumProfile(
            double vmaxF, double vmaxS, double vmaxD,
            double amaxF, double amaxS, double amaxD,
            double dmaxF, double dmaxS, double dmaxD
    ) {

        this.vmaxF = vmaxF;
        this.vmaxS = vmaxS;
        this.velocityShape = solveShapeExponent(vmaxF, vmaxS, vmaxD);

        this.amaxF = amaxF;
        this.amaxS = amaxS;
        this.accelerationShape = solveShapeExponent(amaxF, amaxS, amaxD);

        this.dmaxF = dmaxF;
        this.dmaxS = dmaxS;
        this.decelerationShape = solveShapeExponent(dmaxF, dmaxS, dmaxD);
    }

    /// @param angle is the direction relative to the robot's forward axis, in radians
    /// @return the max velocity in that direction, in inches per second
    public double getMaxVelocity(double angle) {
        return MathHelper.lameValue(angle, vmaxF, vmaxS, velocityShape);
    }

    /// @param angle is the direction relative to the robot's forward axis, in radians
    /// @return the max acceleration in that direction, in inches per second squared
    public double getMaxAcceleration(double angle) {
        return MathHelper.lameValue(angle, amaxF, amaxS, accelerationShape);
    }

    /// @param angle is the direction relative to the robot's forward axis, in radians
    /// @return the max deceleration in that direction, in inches per second squared
    public double getMaxDeceleration(double angle) {
        return MathHelper.lameValue(angle, dmaxF, dmaxS, decelerationShape);
    }

    private double solveShapeExponent(double a, double b, double diagonalValue) {

        double lo = 0.01;
        double hi = 100;

        final int bisectionIterations = 80; //~60 is enough but a larger value is used to ensure accuracy

        for (int i = 0; i < bisectionIterations; i++) {

            double mid = (lo + hi) / 2d;
            double predicted = MathHelper.lameValue(Math.PI / 4d, a, b, mid);

            if (predicted < diagonalValue) lo = mid;
            else hi = mid;
        }

        return (lo + hi) / 2d;
    }

}
package org.firstinspires.ftc.teamcode.util;

import java.util.Arrays;

public final class Ramp {

    public double ENOUGH_TIME_CONSTANTS;

    public Ramp(double enoughTimeConstants) {
        ENOUGH_TIME_CONSTANTS = enoughTimeConstants;
    }

    public Ramp() {
        this (6);
    }

    private static final int MOST_SAMPLES_READ = 256;

    private double[] times = new double[256];
    private double[] distances = new double[256];

    private int count;

    private int solvedAt = -1;
    private double[] solved;

    public void add(double time, double distance) {

        if (count == times.length) {

            times = Arrays.copyOf(times, count * 2);
            distances = Arrays.copyOf(distances, count * 2);
        }

        times[count] = time;
        distances[count] = distance;

        count++;
    }

    /// @return {vmax, amax}, or null while there is not enough of the curve to read
    public double[] fit() {

        double[] found = solve();

        return found == null ? null : new double[] {found[0], found[0] / found[1]};
    }

    /// @return the speed the curve says it was doing at the last sample, which is the
    /// speed braking actually starts from (0 if it can't be determined)
    public double speed() {

        double[] found = solve();

        return found == null ? 0 : found[0] * (1 - Math.exp(-times[count - 1] / found[1]));
    }

    /// @return how many time constants long the run is (0 if it can't be determined)
    public double timeConstants() {

        double[] found = solve();

        return found == null ? 0 : times[count - 1] / found[1];
    }

    private double[] solve() {

        //vmax * (t - tau * (1 - e^-t/tau))
        //exponential first-order modeling

        if (count == solvedAt) return solved;

        solvedAt = count;
        solved = null;

        if (count < 8 || times[count - 1] <= 0) return null;

        double low = 0.02, high = 5;

        double bestTau = -1, best = -1;

        for (int i = 0; i <= 24; i++) {

            double tau = low * Math.pow(high / low, i / 24d);
            double[] at = atTau(tau);

            if (at != null && at[1] > best) {

                best = at[1];
                bestTau = tau;
            }
        }

        if (bestTau < 0) return null;

        double from = bestTau / 1.6, to = bestTau * 1.6;
        double golden = (Math.sqrt(5) - 1) / 2;

        for (int i = 0; i < 40; i++) {

            double left = to - golden * (to - from), right = from + golden * (to - from);

            double[] atLeft = atTau(left), atRight = atTau(right);

            if (atLeft == null || atRight == null) break;

            if (atLeft[1] > atRight[1]) to = right;
            else from = left;
        }

        double tau = (from + to) / 2;
        double[] at = atTau(tau);

        if (at == null || at[0] <= 0 || tau <= 0) return null;

        solved = new double[] {at[0], tau}; //{vmax, tau}

        return solved;
    }

    private double[] atTau(double tau) {

        int stride = Math.max(1, count / MOST_SAMPLES_READ);

        double together = 0, spread = 0;

        for (int i = 0; i < count; i += stride) {

            double shape = times[i] - tau * (1 - Math.exp(-times[i] / tau));

            together += distances[i] * shape;
            spread += shape * shape;
        }

        if (spread <= 0) return null;

        return new double[] {together / spread, together * together / spread};
    }

    //Thank you Euler
    //Thank you Legendre
    //Thank you Gauss
    //Thank you Kiefer (way too underrated)

}

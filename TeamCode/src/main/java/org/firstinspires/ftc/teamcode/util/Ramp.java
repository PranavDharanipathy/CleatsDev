package org.firstinspires.ftc.teamcode.util;

import java.util.Arrays;

public final class Ramp {

    //only means something once the run has gone on this many time constants
    public double ENOUGH_TIME_CONSTANTS;

    public Ramp(double enoughTimeConstants) {
        ENOUGH_TIME_CONSTANTS = enoughTimeConstants;
    }

    public Ramp() {
        this (6);
    }

    private double[] times = new double[256];
    private double[] distances = new double[256];

    private int count;

    public void add(double time, double distance) {

        if (count == times.length) {

            times = Arrays.copyOf(times, count * 2);
            distances = Arrays.copyOf(distances, count * 2);
        }

        times[count] = time;
        distances[count] = distance;

        count++;
    }

    /// @return {vmax, amax}, or null while there is not enough of a straight part to read
    public double[] fit() {

        if (count < 8) return null;

        double[] line = null;
        double from = times[count - 1] / 2;

        //the straight part starts a few time constants in which needs the fit itself to know
        for (int i = 0; i < 4; i++) {

            double[] next = straightFrom(from);

            if (next == null) break;

            line = next;

            double tau = -line[1] / line[0];

            if (tau <= 0 || 4 * tau >= times[count - 1]) break;

            from = 4 * tau;
        }

        if (line == null || line[0] <= 0 || line[1] >= 0) return null;

        return new double[] {line[0], -line[0] * line[0] / line[1]};
    }

    /// @return how many time constants long the run is (0 if it can't be determined)
    public double timeConstants() {

        double[] line = count < 8 ? null : straightFrom(times[count - 1] / 2);

        if (line == null || line[0] <= 0 || line[1] >= 0) return 0;

        return times[count - 1] * line[0] / -line[1];
    }

    private double[] straightFrom(double from) {

        double sumTime = 0, sumDistance = 0;
        int n = 0;

        for (int i = 0; i < count; i++) {

            if (times[i] < from) continue;

            sumTime += times[i];
            sumDistance += distances[i];

            n++;
        }

        if (n < 4) return null;

        double meanTime = sumTime / n, meanDistance = sumDistance / n;

        double spread = 0, together = 0;

        for (int i = 0; i < count; i++) {

            if (times[i] < from) continue;

            double offset = times[i] - meanTime;

            spread += offset * offset;
            together += offset * (distances[i] - meanDistance);
        }

        if (spread <= 0) return null;

        double slope = together / spread;

        return new double[] {slope, meanDistance - slope * meanTime}; //{slope, intercept}
    }

}

package org.firstinspires.ftc.teamcode.util;

/// Used to deal with noise in data.
/// <p>
/// 1D Kalman Filter with outlier rejection.
public class NoiseKalmanFilter {

    private double kalmanGain, filteredData;

    private double p, /*prediction step*/ q, /*uncertainty step*/ r /*data uncertainty*/, outlierThresholdMultiplier;

    private boolean useProvidedDt;

    public NoiseKalmanFilter(boolean useProvidedDt) {

        this.useProvidedDt = useProvidedDt;

        reset();

        currTime = 0;
        startTime = System.nanoTime() * 1e-9;

        areParametersSet = false;
    }

    public NoiseKalmanFilter() {
        this (false);
    }

    private boolean areParametersSet;

    public void setParameters(double q, double r, double outlierThresholdMultiplier) {

        this.q = q;
        this.r = r;

        this.outlierThresholdMultiplier = outlierThresholdMultiplier;

        areParametersSet = true;
    }

    /// If and when discontinuities occur in data, the kalman filter must be reset.
    public void reset() {

        kalmanGain = 0;
        filteredData = 0;
        rawData = 0;

        p = 0;
    }

    private double prevTime, currTime, startTime;

    public void update(double data, double providedDt) {

        if (!areParametersSet) throw new RuntimeException("Parameters weren't set!");

        double dt;

        if (useProvidedDt) {
            dt = providedDt;
        }
        else {

            prevTime = currTime;
            currTime = (System.nanoTime() * 1e-9) - startTime;
            dt = currTime - prevTime;
        }

        if (dt <= 0) return;

        p += q * dt;

        rawData = data;

        double innovation = rawData - filteredData;

        double nominalJump = p + r;

        double deviationRaw = Math.abs(innovation);
        double nominalDeviation = nominalJump > 0 ? Math.sqrt(nominalJump) : 0;

        double outlierThreshold = outlierThresholdMultiplier * nominalDeviation;

        double correctedR;

        if (outlierThreshold > 0 && deviationRaw > outlierThreshold) {

            double deviationRawOutlierThresholdRatio = deviationRaw / outlierThreshold;

            correctedR = r * (deviationRawOutlierThresholdRatio * deviationRawOutlierThresholdRatio);
        }
        else correctedR = r;

        double correctedJump = p + correctedR;

        kalmanGain = correctedJump != 0 ? p / correctedJump : 0;

        filteredData += kalmanGain * innovation;

        p *= (1 - kalmanGain);
    }

    public void update(double data) {
        update(data, 0);
    }

    public double getOutput() {
        return filteredData;
    }

    private double rawData;

    public double getRawData() {
        return rawData;
    }
}

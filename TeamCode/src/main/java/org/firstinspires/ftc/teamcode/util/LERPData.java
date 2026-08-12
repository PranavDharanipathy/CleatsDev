package org.firstinspires.ftc.teamcode.util;

import java.util.Arrays;

public class LERPData {

    /// First indexed value is the data pair, the second is x or y of the data pair.
    public double[][] dataPoints;

    /**
     * @param dataPoint1 minimum
     * @param dataPoint2 maximum
     **/
    public LERPData(double[] dataPoint1, double[] dataPoint2) {
        dataPoints = new double[][] {dataPoint1, dataPoint2};
    }

    public LERPData(double[] inputDataset, double[] outputDataset, double[] bounds) {

        this (
                new double[] {bounds[0], outputDataset[Arrays.asList(inputDataset).indexOf(bounds[0])]},
                new double[] {bounds[1], outputDataset[Arrays.asList(inputDataset).indexOf(bounds[1])]}
        );
    }
}
package org.firstinspires.ftc.teamcode.util;

public class LERP {

    private final double[] inputDataset;
    private final double[] outputDataset;

    public LERP(double[] inputDataset, double[] outputDataset) {
        this.inputDataset = inputDataset;
        this.outputDataset = outputDataset;
    }

    private double getOutput(double dataPointInput) {

        //poles and beyond poles handling
        if (dataPointInput <= inputDataset[0]) return outputDataset[0];
        if (dataPointInput >= inputDataset[inputDataset.length - 1]) return outputDataset[outputDataset.length - 1];

        //getting bounds (inputs) of the inputted data point
        double[] bounds = MathHelper.findBoundingValues(inputDataset, dataPointInput);

        return MathHelper.lerp(dataPointInput, new LERPData(inputDataset, outputDataset, bounds));
    }
}
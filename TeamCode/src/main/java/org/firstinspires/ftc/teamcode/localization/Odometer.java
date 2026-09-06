package org.firstinspires.ftc.teamcode.localization;

public class Odometer {

    private Odometer(double inchesPerTick) {
        this.inchesPerTick = inchesPerTick;
    }

    private final double inchesPerTick;

    public static Odometer createCustomPod(double ticksPerRevolution, double wheelDiameterInches) {
        return new Odometer(Math.PI * (wheelDiameterInches / ticksPerRevolution));
    }

    public static Odometer createGobildaSwingArmPod() {
        return new Odometer(0.0029684340033919307);
    }

    public static Odometer createGobildaFourBarPod() {
        return new Odometer(0.0019789560022612871);
    }

    public static Odometer createSwyftLinearPod() {
        return new Odometer(0.0011474659436939835);
    }

    public double getInchesPerTick() {
        return inchesPerTick;
    }
}

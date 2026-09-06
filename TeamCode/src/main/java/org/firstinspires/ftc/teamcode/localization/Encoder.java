package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Encoder {

    public enum Direction {
        FORWARD(1),
        REVERSE(-1);

        private final int multiplier;

        Direction(int multiplier) {
            this.multiplier = multiplier;
        }

        public int getMultiplier() {
            return multiplier;
        }
    }

    private final DcMotorEx motor;
    private final double inchesPerTick;

    private Direction direction = Direction.FORWARD;

    private int currentPositionTicks;
    private int lastPositionTicks;

    private double positionInches;
    private double deltaInches;

    public Encoder(DcMotorEx motor, Odometer odometer) {

        this.motor = motor;

        this.inchesPerTick = odometer.getInchesPerTick();

        currentPositionTicks = motor.getCurrentPosition();
        lastPositionTicks = currentPositionTicks;
    }

    public void update() {

        currentPositionTicks = motor.getCurrentPosition() * direction.getMultiplier();

        int deltaTicks = currentPositionTicks - lastPositionTicks;

        deltaInches = deltaTicks * inchesPerTick;
        positionInches = currentPositionTicks * inchesPerTick;

        lastPositionTicks = currentPositionTicks;
    }

    public void reset() {

        currentPositionTicks = motor.getCurrentPosition() * direction.getMultiplier();
        lastPositionTicks = currentPositionTicks;

        positionInches = 0;
        deltaInches = 0;
    }

    public void setDirection(Direction direction) {

        this.direction = direction;

        currentPositionTicks = motor.getCurrentPosition() * direction.getMultiplier();
        lastPositionTicks = currentPositionTicks;
    }

    public Direction getDirection() {
        return direction;
    }

    public double getPositionInches() {
        return positionInches;
    }

    public double getDeltaInches() {
        return deltaInches;
    }

    public int getCurrentPositionTicks() {
        return currentPositionTicks;
    }

    public int getRawPositionTicks() {
        return motor.getCurrentPosition();
    }

    public double getInchesPerTick() {
        return inchesPerTick;
    }
}

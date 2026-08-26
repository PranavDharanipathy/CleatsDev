package org.firstinspires.ftc.teamcode.following.chassis;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class SlipPreventedWheel {

    // not bad kid

    private final DcMotor wheel;

    private double prevPower, currPower;

    private final double rampRate;

    public SlipPreventedWheel(DcMotor wheel, double rampRate) {

        this.wheel = wheel;

        this.rampRate = rampRate;

        prevPower = 0;
        currPower = 0;
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        wheel.setDirection(direction);
    }

    public void setPower(double requestedPower, double dt) {

        prevPower = currPower;

        double maxDelta = rampRate * Math.max(0, dt);

        if (prevPower >= 0) {
            currPower = requestedPower > prevPower
                    ? Math.min(requestedPower, prevPower + maxDelta)
                    : requestedPower;
        }
        else {
            currPower = requestedPower < prevPower
                    ? Math.max(requestedPower, prevPower - maxDelta)
                    : requestedPower;
        }

        wheel.setPower(currPower);
    }

    public void setPowerBypassRamp(double power) {

        prevPower = currPower;
        currPower = power;

        wheel.setPower(currPower);
    }

    public double getPower() {
        return currPower;
    }
}
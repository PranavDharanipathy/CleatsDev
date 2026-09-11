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

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        wheel.setZeroPowerBehavior(zeroPowerBehavior);
    }

    /// Slip happens when torque grows, whichever way the wheel is turning, so the ramp
    /// limits magnitude. Reversals and any drop in magnitude still land immediately.
    public void setPower(double requestedPower, double dt) {

        prevPower = currPower;

        double maxDelta = rampRate * Math.max(0, dt);

        boolean sameDirection = requestedPower * prevPower >= 0;

        if (sameDirection && Math.abs(requestedPower) > Math.abs(prevPower)) {
            currPower = Math.signum(requestedPower) * Math.min(Math.abs(requestedPower), Math.abs(prevPower) + maxDelta);
        }
        else currPower = requestedPower;

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

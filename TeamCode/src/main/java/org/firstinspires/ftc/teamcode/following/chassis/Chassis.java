package org.firstinspires.ftc.teamcode.following.chassis;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/// Mecanum drivetrain
public class Chassis {

    private final SlipPreventedWheel lf, rf, lb, rb;

    /// @param motorDeviceNames should contain the names in the order of left front, right front, left back, right back.
    /// @param motorDirections should contain the names in the order of left front, right front, left back, right back.
    public Chassis(HardwareMap hardwareMap, String[] motorDeviceNames, DcMotorSimple.Direction[] motorDirections, double rampRate) {

        lf = new SlipPreventedWheel(hardwareMap.get(DcMotor.class, motorDeviceNames[0]), rampRate);
        rf = new SlipPreventedWheel(hardwareMap.get(DcMotor.class, motorDeviceNames[1]), rampRate);
        lb = new SlipPreventedWheel(hardwareMap.get(DcMotor.class, motorDeviceNames[2]), rampRate);
        rb = new SlipPreventedWheel(hardwareMap.get(DcMotor.class, motorDeviceNames[3]), rampRate);

        lf.setDirection(motorDirections[0]);
        rf.setDirection(motorDirections[1]);
        lb.setDirection(motorDirections[2]);
        rb.setDirection(motorDirections[3]);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setDrivePower(double forward, double strafe, double turn, double dt) {

        double lfPower = forward + strafe + turn;
        double rfPower = forward - strafe - turn;
        double lbPower = forward - strafe + turn;
        double rbPower = forward + strafe - turn;

        double maxMagnitude = Math.max(Math.max(Math.abs(lfPower), Math.abs(rfPower)), Math.max(Math.abs(lbPower), Math.abs(rbPower)));
        double scale = maxMagnitude > 1d ? 1d / maxMagnitude : 1d;

        lf.setPower(lfPower * scale, dt);
        rf.setPower(rfPower * scale, dt);
        lb.setPower(lbPower * scale, dt);
        rb.setPower(rbPower * scale, dt);
    }

    public void setDrivePowerBypassRamp(double forward, double strafe, double turn) {

        double lfPower = forward + strafe + turn;
        double rfPower = forward - strafe - turn;
        double lbPower = forward - strafe + turn;
        double rbPower = forward + strafe - turn;

        double maxMagnitude = Math.max(Math.max(Math.abs(lfPower), Math.abs(rfPower)), Math.max(Math.abs(lbPower), Math.abs(rbPower)));
        double scale = maxMagnitude > 1d ? 1d / maxMagnitude : 1d;

        lf.setPowerBypassRamp(lfPower * scale);
        rf.setPowerBypassRamp(rfPower * scale);
        lb.setPowerBypassRamp(lbPower * scale);
        rb.setPowerBypassRamp(rbPower * scale);
    }

    public double getLFPower() { return lf.getPower(); }

    public double getRFPower() { return rf.getPower(); }

    public double getLBPower() { return lb.getPower(); }

    public double getRBPower() { return rb.getPower(); }
}

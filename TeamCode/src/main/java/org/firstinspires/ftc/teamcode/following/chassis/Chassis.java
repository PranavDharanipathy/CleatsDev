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
    }

    public void setDrivePower(double forward, double strafe, double turn, double dt) {

        lf.setPower(forward + strafe + turn, dt);
        rf.setPower(forward - strafe - turn, dt);
        lb.setPower(forward - strafe + turn, dt);
        rb.setPower(forward + strafe - turn, dt);
    }

    public void setDrivePowerBypassRamp(double forward, double strafe, double turn) {

        lf.setPowerBypassRamp(forward + strafe + turn);
        rf.setPowerBypassRamp(forward - strafe - turn);
        lb.setPowerBypassRamp(forward - strafe + turn);
        rb.setPowerBypassRamp(forward + strafe - turn);
    }

    public double getLFPower() { return lf.getPower(); }

    public double getRFPower() { return rf.getPower(); }

    public double getLBPower() { return lb.getPower(); }

    public double getRBPower() { return rb.getPower(); }
}

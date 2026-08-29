package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.following.PathController;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.Constants;

//@Config
@TeleOp(group = "Cleats Tuning")
public class ForwardTest extends LinearOpMode {

    public static double DECEL_RAMP_DURATION = 2;

    private PathController pc;

    @Override
    public void runOpMode() {

        pc = Constants.getPathController();

        telemetry.addLine("Press A to begin braking.");
        telemetry.addLine("The robot will move forward. Let the robot cruise for as long as possible before braking.");
        telemetry.update();

        waitForStart();

        double peakAccel = 0;
        double peakForwardSpeed = 0;

        while (opModeIsActive() && !gamepad1.a) {

            pc.getChassis().setDrivePowerBypassRamp(1, 0, 0);

            pc.update();

            Pose vel = pc.getFinalLocalizer().getVelocity();
            Pose accel = pc.getFinalLocalizer().getAcceleration();

            double accelMagnitude = Math.hypot(accel.x, accel.y);
            double speed = Math.hypot(vel.x, vel.y);

            peakAccel = Math.max(peakAccel, accelMagnitude);
            peakForwardSpeed = Math.max(peakForwardSpeed, speed);

            telemetry.addData("measured accel", accelMagnitude);
            telemetry.addData("peak accel so far", peakAccel);
            telemetry.addData("peak speed so far", peakForwardSpeed);
            telemetry.update();
        }

        double decelStartTime = getRuntime();
        double peakDecel = 0;

        while (opModeIsActive()) {

            double t = getRuntime() - decelStartTime;
            if (t > DECEL_RAMP_DURATION) break;

            double commandedPower = -t / DECEL_RAMP_DURATION;
            pc.getChassis().setDrivePowerBypassRamp(commandedPower, 0, 0);

            pc.update();

            Pose pose = pc.getFinalLocalizer().getPose();
            Pose vel = pc.getFinalLocalizer().getVelocity();
            Pose accel = pc.getFinalLocalizer().getAcceleration();

            double accelMagnitude = Math.hypot(accel.x, accel.y);
            peakDecel = Math.max(peakDecel, accelMagnitude);

            double forwardVelocity = vel.x * Math.cos(pose.heading) + vel.y * Math.sin(pose.heading); //forward velocity relative to the robot (prevents positional and heading drift from messing up readings)
            if (forwardVelocity < 0) break;

            telemetry.addData("decel commanded power", commandedPower);
            telemetry.addData("measured decel", accelMagnitude);
            telemetry.addData("peak decel so far", peakDecel);
            telemetry.update();
        }

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);

        telemetry.addLine("=== FORWARD RESULTS ===");
        telemetry.addData("amaxF (in/s^2)", peakAccel);
        telemetry.addData("dmaxF (in/s^2)", peakDecel);
        telemetry.addData("vmaxF (in/s)", peakForwardSpeed);
        telemetry.update();

        while (opModeIsActive()) ;
    }
}
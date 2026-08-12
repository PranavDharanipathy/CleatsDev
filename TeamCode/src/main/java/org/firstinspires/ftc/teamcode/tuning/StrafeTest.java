package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.Constants;

//@Config
@TeleOp(group = "Cleats Tuning")
public class StrafeTest extends LinearOpMode {

    public static double DECEL_RAMP_DURATION = 2;

    @Override
    public void runOpMode() {

        telemetry.addLine("Press A to begin braking.");
        telemetry.addLine("The robot will strafe right. Let the robot cruise for as long as possible before braking.");
        telemetry.update();

        waitForStart();

        double peakAccel = 0;
        double peakStrafeSpeed = 0;

        while (opModeIsActive() && !gamepad1.a) {

            Constants.getPathController().getChassis().setDrivePowerBypassRamp(0, 1, 0);

            Constants.getPathController().update();

            Pose vel = Constants.getPathController().getFinalLocalizer().getVelocity();
            Pose accel = Constants.getPathController().getFinalLocalizer().getAcceleration();

            double accelMagnitude = Math.hypot(accel.x, accel.y);
            double speed = Math.hypot(vel.x, vel.y);

            peakAccel = Math.max(peakAccel, accelMagnitude);
            peakStrafeSpeed = Math.max(peakStrafeSpeed, speed);

            telemetry.addData("measured accel", accelMagnitude);
            telemetry.addData("peak accel so far", peakAccel);
            telemetry.addData("peak speed so far", peakStrafeSpeed);
            telemetry.update();
        }

        double decelStartTime = getRuntime();
        double peakDecel = 0;

        while (opModeIsActive()) {

            double t = getRuntime() - decelStartTime;
            if (t > DECEL_RAMP_DURATION) break;

            double commandedPower = -t / DECEL_RAMP_DURATION;
            Constants.getPathController().getChassis().setDrivePowerBypassRamp(0, commandedPower, 0);

            Constants.getPathController().update();

            Pose pose = Constants.getPathController().getFinalLocalizer().getPose();
            Pose vel = Constants.getPathController().getFinalLocalizer().getVelocity();
            Pose accel = Constants.getPathController().getFinalLocalizer().getAcceleration();

            double accelMagnitude = Math.hypot(accel.x, accel.y);
            peakDecel = Math.max(peakDecel, accelMagnitude);

            double strafeVelocity = vel.x * Math.sin(pose.heading) - vel.y * Math.cos(pose.heading);
            if (strafeVelocity < 0) break;

            telemetry.addData("decel commanded power", commandedPower);
            telemetry.addData("measured decel", accelMagnitude);
            telemetry.addData("peak decel so far", peakDecel);
            telemetry.update();
        }

        Constants.getPathController().getChassis().setDrivePowerBypassRamp(0, 0, 0);

        telemetry.addLine("=== STRAFE RESULTS ===");
        telemetry.addData("amaxS (in/s^2)", peakAccel);
        telemetry.addData("dmaxS (in/s^2)", peakDecel);
        telemetry.addData("vmaxS (in/s)", peakStrafeSpeed);
        telemetry.update();

        while (opModeIsActive()) ;
    }
}
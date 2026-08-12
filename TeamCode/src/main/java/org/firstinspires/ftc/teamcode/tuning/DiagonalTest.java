package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.Constants;

//@Config
@TeleOp(group = "Cleats Tuning")
public class DiagonalTest extends LinearOpMode {

    public static double DECEL_RAMP_DURATION = 2;

    @Override
    public void runOpMode() {

        telemetry.addLine("Press A to begin braking.");
        telemetry.addLine("The robot will strafe diagonally forward and right. Let the robot cruise for as long as possible before braking.");
        telemetry.update();

        waitForStart();

        double peakAccel = 0;
        double peakDiagonalSpeed = 0;

        while (opModeIsActive() && !gamepad1.a) {

            Constants.getPathController().getChassis().setDrivePowerBypassRamp(1, 1, 0);

            Constants.getPathController().update();

            Pose vel = Constants.getPathController().getFinalLocalizer().getVelocity();
            Pose accel = Constants.getPathController().getFinalLocalizer().getAcceleration();

            double accelMagnitude = Math.hypot(accel.x, accel.y);
            double speed = Math.hypot(vel.x, vel.y);

            peakAccel = Math.max(peakAccel, accelMagnitude);
            peakDiagonalSpeed = Math.max(peakDiagonalSpeed, speed);

            telemetry.addData("measured accel", accelMagnitude);
            telemetry.addData("peak accel so far", peakAccel);
            telemetry.addData("peak speed so far", peakDiagonalSpeed);
            telemetry.update();
        }

        double decelStartTime = getRuntime();
        double peakDecel = 0;

        while (opModeIsActive()) {

            double t = getRuntime() - decelStartTime;
            if (t > DECEL_RAMP_DURATION) break;

            double commandedPower = -t / DECEL_RAMP_DURATION;
            Constants.getPathController().getChassis().setDrivePowerBypassRamp(commandedPower, commandedPower, 0);

            Constants.getPathController().update();

            Pose pose = Constants.getPathController().getFinalLocalizer().getPose();
            Pose vel = Constants.getPathController().getFinalLocalizer().getVelocity();
            Pose accel = Constants.getPathController().getFinalLocalizer().getAcceleration();

            double accelMagnitude = Math.hypot(accel.x, accel.y);
            peakDecel = Math.max(peakDecel, accelMagnitude);

            double forward = vel.x * Math.cos(pose.heading) + vel.y * Math.sin(pose.heading);
            double strafe = vel.x * Math.sin(pose.heading) - vel.y * Math.cos(pose.heading);
            if (forward + strafe /*diagonal*/ < 0) break;

            telemetry.addData("decel commanded power", commandedPower);
            telemetry.addData("measured decel", accelMagnitude);
            telemetry.addData("peak decel so far", peakDecel);
            telemetry.update();
        }

        Constants.getPathController().getChassis().setDrivePowerBypassRamp(0, 0, 0);

        telemetry.addLine("=== DIAGONAL RESULTS ===");
        telemetry.addData("amaxD (in/s^2)", peakAccel);
        telemetry.addData("dmaxD (in/s^2)", peakDecel);
        telemetry.addData("vmaxD (in/s)", peakDiagonalSpeed);
        telemetry.update();

        while (opModeIsActive()) ;
    }
}
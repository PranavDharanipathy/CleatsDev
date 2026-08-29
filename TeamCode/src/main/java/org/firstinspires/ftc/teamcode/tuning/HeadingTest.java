package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.following.PathController;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.Constants;

//@Config
@TeleOp(group = "Cleats Tuning")
public class HeadingTest extends LinearOpMode {

    public static double DECEL_RAMP_DURATION = 2;

    private PathController pc;

    @Override
    public void runOpMode() {

        pc = Constants.getPathController();

        telemetry.addLine("Press A to begin braking.");
        telemetry.addLine("The robot will spin clockwise. Let the robot reach full speed before braking.");
        telemetry.update();

        waitForStart();

        double peakAccel = 0;
        double peakAngularSpeed = 0;

        while (opModeIsActive() && !gamepad1.a) {

            pc.getChassis().setDrivePowerBypassRamp(0, 0, 1);

            pc.update();

            Pose vel = pc.getFinalLocalizer().getVelocity();
            Pose accel = pc.getFinalLocalizer().getAcceleration();

            double accelMagnitude = Math.abs(accel.heading);
            double angularSpeed = Math.abs(vel.heading);

            peakAccel = Math.max(peakAccel, accelMagnitude);
            peakAngularSpeed = Math.max(peakAngularSpeed, angularSpeed);

            telemetry.addData("measured accel", accelMagnitude);
            telemetry.addData("peak accel so far", peakAccel);
            telemetry.addData("peak angular speed so far", peakAngularSpeed);
            telemetry.update();
        }

        double decelStartTime = getRuntime();
        double peakDecel = 0;

        while (opModeIsActive()) {

            double t = getRuntime() - decelStartTime;
            if (t > DECEL_RAMP_DURATION) break;

            double commandedPower = -t / DECEL_RAMP_DURATION;
            pc.getChassis().setDrivePowerBypassRamp(0, 0, commandedPower);

            pc.update();

            Pose vel = pc.getFinalLocalizer().getVelocity();
            Pose accel = pc.getFinalLocalizer().getAcceleration();

            double accelMagnitude = Math.abs(accel.heading);
            peakDecel = Math.max(peakDecel, accelMagnitude);

            if (vel.heading < 0) break;

            telemetry.addData("decel commanded power", commandedPower);
            telemetry.addData("measured decel", accelMagnitude);
            telemetry.addData("peak decel so far", peakDecel);
            telemetry.update();
        }

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);

        telemetry.addLine("=== HEADING RESULTS ===");
        telemetry.addData("amaxH (rad/s^2)", peakAccel);
        telemetry.addData("dmaxH (rad/s^2)", peakDecel);
        telemetry.addData("vmaxH (rad/s)", peakAngularSpeed);
        telemetry.update();

        while (opModeIsActive()) ;
    }
}
package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.following.PathController;
import org.firstinspires.ftc.teamcode.util.MathHelper;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Ramp;

//@Config
@TeleOp(group = "Cleats Tuning")
public class ForwardTest extends LinearOpMode {

    private PathController pc;

    @Override
    public void runOpMode() {

        pc = Constants.getPathController(hardwareMap);

        telemetry.addLine("The robot will move forward. Cruise until it says A, then press A to begin braking.");
        telemetry.update();

        waitForStart();

        pc.update();

        Pose start = pc.getPose().copy();

        Ramp ramp = new Ramp();

        double begin = getRuntime();

        while (opModeIsActive() && !gamepad1.a) {

            pc.getChassis().setDrivePowerBypassRamp(1, 0, 0);

            pc.update();

            Pose pose = pc.getPose();

            ramp.add(getRuntime() - begin, Math.hypot(pose.x - start.x, pose.y - start.y));

            double constants = ramp.timeConstants();

            telemetry.addData("cruised", "%.1f of %.1f time constants", constants, ramp.ENOUGH_TIME_CONSTANTS);
            telemetry.addLine(constants >= ramp.ENOUGH_TIME_CONSTANTS ? "A to brake" : "keep cruising");
            telemetry.update();
        }

        double[] fit = ramp.fit();
        double atBrake = ramp.speed();

        pc.update();

        Pose brakeStart = pc.getPose().copy();

        double runX = brakeStart.x - start.x, runY = brakeStart.y - start.y;
        double run = Math.hypot(runX, runY);

        double distance = 0;

        if (run > 0) {

            double alongX = runX / run, alongY = runY / run;

            while (opModeIsActive()) {

                pc.getChassis().setDrivePowerBypassRamp(-1, 0, 0);

                pc.update();

                Pose pose = pc.getPose();

                //the stopping distance is the furthest it got, so it's done once it comes back
                double travelled = (pose.x - brakeStart.x) * alongX + (pose.y - brakeStart.y) * alongY;

                if (travelled < distance) break;

                distance = travelled;

                telemetry.addData("braking", "%.2f in", distance);
                telemetry.update();
            }
        }

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);

        if (fit == null) {

            telemetry.addLine("The run never settled into a steady cruise, so there is nothing to read off it.");
            telemetry.update();

            while (opModeIsActive()) ;
            return;
        }

        double dmax = distance > 0 ? atBrake * atBrake / (2 * distance) : 0;

        telemetry.addLine("=== FORWARD RESULTS ===");
        telemetry.addData("vmaxF (in/s)", fit[0]);
        telemetry.addData("amaxF (in/s^2)", Math.min(fit[1], dmax));
        telemetry.addData("dmaxF (in/s^2)", dmax);
        telemetry.update();

        while (opModeIsActive()) ;
    }
}

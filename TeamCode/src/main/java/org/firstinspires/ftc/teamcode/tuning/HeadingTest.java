package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.following.PathController;
import org.firstinspires.ftc.teamcode.util.MathHelper;
import org.firstinspires.ftc.teamcode.util.Ramp;

//@Config
@TeleOp(group = "Cleats Tuning")
public class HeadingTest extends LinearOpMode {

    private PathController pc;

    @Override
    public void runOpMode() {

        pc = Constants.getPathController(hardwareMap);

        telemetry.addLine("The robot will spin in place. Let it spin until it says A, then press A to begin braking.");
        telemetry.update();

        waitForStart();

        pc.update();

        double previous = pc.getHeading();
        double rotated = 0;

        Ramp ramp = new Ramp();

        double begin = getRuntime();

        while (opModeIsActive() && !gamepad1.a) {

            pc.getChassis().setDrivePowerBypassRamp(0, 0, 1);

            pc.update();

            double heading = pc.getHeading();
            rotated += MathHelper.normalizeAngleRad(heading - previous);
            previous = heading;

            ramp.add(getRuntime() - begin, Math.abs(rotated));

            double constants = ramp.timeConstants();

            telemetry.addData("spun", "%.1f of %.1f time constants", constants, ramp.ENOUGH_TIME_CONSTANTS);
            telemetry.addLine(constants >= ramp.ENOUGH_TIME_CONSTANTS ? "A to brake" : "keep spinning");
            telemetry.update();
        }

        double[] fit = ramp.fit();
        double atBrake = ramp.speed();

        double spin = Math.signum(rotated);

        double braked = 0;
        double angle = 0;

        if (spin != 0) {

            pc.update();
            previous = pc.getHeading();

            while (opModeIsActive()) {

                pc.getChassis().setDrivePowerBypassRamp(0, 0, -1);

                pc.update();

                double heading = pc.getHeading();
                braked += MathHelper.normalizeAngleRad(heading - previous) * spin;
                previous = heading;

                //the stopping angle is the furthest it got, so it's done once it spins back
                if (braked < angle) break;

                angle = braked;

                telemetry.addData("braking", "%.4f rad", angle);
                telemetry.update();
            }
        }

        pc.getChassis().setDrivePowerBypassRamp(0, 0, 0);

        if (fit == null) {

            telemetry.addLine("The spin never settled into a steady rate, so there is nothing to read off it.");
            telemetry.update();

            while (opModeIsActive()) ;
            return;
        }

        double dmax = angle > 0 ? atBrake * atBrake / (2 * angle) : 0;

        telemetry.addLine("=== HEADING RESULTS ===");
        telemetry.addData("vmaxH (rad/s)", fit[0]);
        telemetry.addData("amaxH (rad/s^2)", Math.min(fit[1], dmax));
        telemetry.addData("dmaxH (rad/s^2)", dmax);
        telemetry.update();

        while (opModeIsActive()) ;
    }
}

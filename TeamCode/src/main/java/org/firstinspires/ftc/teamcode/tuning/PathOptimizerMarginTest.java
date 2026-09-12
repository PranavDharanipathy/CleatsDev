package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.following.PathController;
import org.firstinspires.ftc.teamcode.path.HermiteSpline;
import org.firstinspires.ftc.teamcode.path.Movement;
import org.firstinspires.ftc.teamcode.util.Pose;

//@Config
@TeleOp(group = "Cleats Tuning")
public class PathOptimizerMarginTest extends LinearOpMode {

    public static double SIZE = 48;

    public static double DRIVE_TIMEOUT = 15;
    public static double RETURN_TIMEOUT = 12;

    private PathController pc;
    private Pose home;

    @Override
    public void runOpMode() {

        pc = Constants.getPathController(hardwareMap);

        telemetry.addLine("PathOptimizer margin.");
        telemetry.addLine("Clear a " + ((int) (SIZE * 1.5)) + " inch square in front and to the left of the robot.");
        telemetry.addLine("The robot will drive in four shapes and tell you how far it moved away from the line.");
        telemetry.update();

        waitForStart();

        pc.update();
        home = pc.getPose().copy();

        String[] names = {"straight", "gentle S", "hard bend", "hairpin"};
        double[] worst = new double[names.length];

        for (int i = 0; i < names.length && opModeIsActive(); i++) {

            worst[i] = drive(names[i], shape(i));
            returnHome();
        }

        double overall = 0;
        for (double error : worst) {
            overall = Math.max(overall, error);
        }

        telemetry.addLine("=== PATH OPTIMIZER MARGIN ===");
        telemetry.addData("worst anywhere (in)", overall);
        telemetry.addData("setMargin", Math.ceil(overall * 4) / 4);
        telemetry.addLine("The margin has to cover this, or the planner will route the robot");
        telemetry.addLine("through gaps the follower cannot actually hold it in.");
        telemetry.update();

        while (opModeIsActive()) ;
    }

    private Movement shape(int which) {

        switch (which) {

            case 0: return new HermiteSpline(at(0, 0), at(SIZE, 0));

            case 1: return new HermiteSpline(at(0, 0), at(SIZE / 2, SIZE / 3), at(SIZE, 0));

            case 2: return new HermiteSpline(at(0, 0), at(SIZE * 0.6, 0), at(SIZE * 0.6, SIZE * 0.6));

            default: return new HermiteSpline(at(0, 0), at(SIZE * 0.8, SIZE * 0.25), at(SIZE * 0.1, SIZE * 0.5));
        }
    }

    private Pose at(double forward, double left) {

        double cos = Math.cos(home.heading), sin = Math.sin(home.heading);

        return new Pose(home.x + forward * cos - left * sin, home.y + forward * sin + left * cos, home.heading);
    }

    private double drive(String name, Movement path) {

        pc.follow(path);

        double started = getRuntime();
        double worst = 0;

        while (opModeIsActive() && pc.isFollowing() && getRuntime() - started < DRIVE_TIMEOUT) {

            pc.update();

            worst = Math.max(worst, path.getPathError(pc.getPose()));

            telemetry.addLine("DRIVING " + name);
            telemetry.addData("off the line now (in)", path.getPathError(pc.getPose()));
            telemetry.addData("worst so far (in)", worst);
            telemetry.update();
        }

        pc.cancel();

        return worst;
    }

    private void returnHome() {

        pc.update();

        if (Math.hypot(pc.getX() - home.x, pc.getY() - home.y) < 2) return;

        pc.follow(new HermiteSpline(pc.getPose().copy(), home.copy()));

        double started = getRuntime();

        while (opModeIsActive() && pc.isFollowing() && getRuntime() - started < RETURN_TIMEOUT) {

            pc.update();

            telemetry.addLine("driving back to the start spot");
            telemetry.update();
        }

        pc.cancel();
    }
}

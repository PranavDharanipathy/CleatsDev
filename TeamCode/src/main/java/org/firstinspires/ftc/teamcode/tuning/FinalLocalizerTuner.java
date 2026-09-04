package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.following.PathController;
import org.firstinspires.ftc.teamcode.following.chassis.Chassis;
import org.firstinspires.ftc.teamcode.localization.FinalLocalizer;

@Config
@TeleOp(group = "Cleats Tuning")
public class FinalLocalizerTuner extends OpMode {

    public static int STAGE = 1;

    public static double Q, R, OUTLIER_THRESHOLD_MULTIPLIER;

    private Chassis chassis;
    private FinalLocalizer finalLocalizer;

    private Telemetry telemetry;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        PathController pc = Constants.getPathController();

        chassis = pc.getChassis();
        finalLocalizer = new FinalLocalizer(pc.getFinalLocalizer().getLocalizer()); //custom copy with same localizer

        //default values
        double[] defaultParams = {1, 1, 1}; // {q, r, outlierThresholdMultiplier}
        finalLocalizer.setNoiseFilterParameters(
                defaultParams, defaultParams, defaultParams,
                defaultParams, defaultParams, defaultParams
        );

        telemetry.addLine("Stage 1: velocity x");
        telemetry.addLine("Stage 2: velocity y");
        telemetry.addLine("Stage 3: velocity heading");
        telemetry.addLine("Stage 4: acceleration x");
        telemetry.addLine("Stage 5: acceleration y");
        telemetry.addLine("Stage 6: acceleration heading");
        telemetry.addLine();
        telemetry.addLine("Please tune in order of stages! First tune velocity, then acceleration, and then jerk.");
        telemetry.addLine("Make sure to change stage # before changing KF parameters once you finish tuning one!");
        telemetry.update();
    }

    @Override
    public void start() {
        telemetry.clearAll();
    }

    @Override
    public void loop() {

        chassis.setDrivePowerBypassRamp(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        switch (STAGE) {

            case 1:

                finalLocalizer.velocityX.setParameters(Q, R, OUTLIER_THRESHOLD_MULTIPLIER);
                break;

            case 2:

                finalLocalizer.velocityY.setParameters(Q, R, OUTLIER_THRESHOLD_MULTIPLIER);
                break;

            case 3:

                finalLocalizer.velocityHeading.setParameters(Q, R, OUTLIER_THRESHOLD_MULTIPLIER);
                break;

            case 4:

                finalLocalizer.accelerationX.setParameters(Q, R, OUTLIER_THRESHOLD_MULTIPLIER);
                break;

            case 5:

                finalLocalizer.accelerationY.setParameters(Q, R, OUTLIER_THRESHOLD_MULTIPLIER);
                break;

            case 6:

                finalLocalizer.accelerationHeading.setParameters(Q, R, OUTLIER_THRESHOLD_MULTIPLIER);
                break;

            default:
                break;
        }

        finalLocalizer.update();

        telemetry.addData("velocityX unfiltered", finalLocalizer.velocityX.getRawData());
        telemetry.addData("velocityX filtered", finalLocalizer.velocityX.getOutput());

        telemetry.addData("velocityY unfiltered", finalLocalizer.velocityY.getRawData());
        telemetry.addData("velocityY filtered", finalLocalizer.velocityY.getOutput());

        telemetry.addData("velocityHeading unfiltered", finalLocalizer.velocityHeading.getRawData());
        telemetry.addData("velocityHeading filtered", finalLocalizer.velocityHeading.getOutput());

        telemetry.addData("accelerationX unfiltered", finalLocalizer.accelerationX.getRawData());
        telemetry.addData("accelerationX filtered", finalLocalizer.accelerationX.getOutput());

        telemetry.addData("accelerationY unfiltered", finalLocalizer.accelerationY.getRawData());
        telemetry.addData("accelerationY filtered", finalLocalizer.accelerationY.getOutput());

        telemetry.addData("accelerationHeading unfiltered", finalLocalizer.accelerationHeading.getRawData());
        telemetry.addData("accelerationHeading filtered", finalLocalizer.accelerationHeading.getOutput());

        telemetry.update();
    }
}

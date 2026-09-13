package org.firstinspires.ftc.teamcode.dribble;

import org.firstinspires.ftc.teamcode.following.PathController;

public class DribblePathController implements Subsystem {

    // "DribblePathController" goes hard

    private static DribblePathController active;

    private final PathController pathController;

    public DribblePathController(PathController pathController) {

        this.pathController = pathController;

        active = this;
    }

    public static DribblePathController getActive() {

        if (active == null) throw new IllegalStateException("No DribblePathController has been built yet, make one before scheduling a FollowPath!");

        return active;
    }

    public PathController getPathController() {
        return pathController;
    }

    @Override
    public void loop() {

        pathController.update();

        //PathController already zeroes on the loop a maneuver ends, this is just a guard
        if (!pathController.isFollowing()) pathController.getChassis().setDrivePowerBypassRamp(0, 0, 0);
    }
}

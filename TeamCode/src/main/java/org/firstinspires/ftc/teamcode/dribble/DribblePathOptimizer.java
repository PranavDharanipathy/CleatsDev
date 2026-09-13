package org.firstinspires.ftc.teamcode.dribble;

import org.firstinspires.ftc.teamcode.following.PathController;
import org.firstinspires.ftc.teamcode.following.chassis.MecanumProfile;
import org.firstinspires.ftc.teamcode.path.HeadingOp;
import org.firstinspires.ftc.teamcode.path.HermiteSpline;
import org.firstinspires.ftc.teamcode.path.Movement;
import org.firstinspires.ftc.teamcode.path.optimization.Obstacle;
import org.firstinspires.ftc.teamcode.path.optimization.PathOptimizer;
import org.firstinspires.ftc.teamcode.path.optimization.RobotFrame;
import org.firstinspires.ftc.teamcode.util.Pose;

import java.util.ArrayList;
import java.util.Arrays;

public class DribblePathOptimizer {

    private static DribblePathOptimizer active;

    private final PathOptimizer pathOptimizer;
    private final ArrayList<Obstacle> obstacles = new ArrayList<>();

    public DribblePathOptimizer(RobotFrame robotFrame, MecanumProfile mecanumProfile, Obstacle... obstacles) {
        this (new PathOptimizer(robotFrame, mecanumProfile), obstacles);
    }

    public DribblePathOptimizer(PathOptimizer pathOptimizer, Obstacle... obstacles) {

        this.pathOptimizer = pathOptimizer;
        this.obstacles.addAll(Arrays.asList(obstacles));

        active = this;
    }

    /// The most recently built one, which is what {@link DriveTo} uses when it is not handed one.
    public static DribblePathOptimizer getActive() {

        if (active == null) throw new IllegalStateException("No DribblePathOptimizer has been built yet, make one before scheduling a DriveTo!");

        return active;
    }

    public DribblePathOptimizer add(Obstacle... toAdd) {

        obstacles.addAll(Arrays.asList(toAdd));
        return this;
    }

    public DribblePathOptimizer remove(Obstacle obstacle) {

        obstacles.remove(obstacle);
        return this;
    }

    public DribblePathOptimizer clear() {

        obstacles.clear();
        return this;
    }

    /// Extra slack kept between the robot and every obstacle in inches.
    public DribblePathOptimizer setMargin(double margin) {

        pathOptimizer.setMargin(margin);
        return this;
    }

    /// Sets how heading will be handled on the paths produced (defaults to efficient heading).
    public DribblePathOptimizer setHeadingOp(HeadingOp headingOp) {

        pathOptimizer.setHeadingOp(headingOp);
        return this;
    }

    public Obstacle[] getObstacles() {
        return obstacles.toArray(new Obstacle[0]);
    }

    public PathOptimizer getPathOptimizer() {
        return pathOptimizer;
    }

    /// @return the quickest way from one pose to the other (null when the robot doesn't fit)
    public Movement pathFrom(Pose from, Pose to) {

        //an unchanged path means the optimizer couldn't find a route
        final Movement refused = new HermiteSpline(from.copy(), from.copy());

        Movement path = pathOptimizer.produceEfficientPath(from.copy(), to.copy(), refused, getObstacles());

        return path == refused ? null : path;
    }
}

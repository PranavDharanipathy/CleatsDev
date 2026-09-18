package org.firstinspires.ftc.teamcode.playmaker;

import org.firstinspires.ftc.teamcode.dribble.Command;
import org.firstinspires.ftc.teamcode.dribble.DeadlineCommand;
import org.firstinspires.ftc.teamcode.dribble.Delay;
import org.firstinspires.ftc.teamcode.dribble.FollowPath;
import org.firstinspires.ftc.teamcode.dribble.ParallelCommand;
import org.firstinspires.ftc.teamcode.dribble.RaceCommand;
import org.firstinspires.ftc.teamcode.dribble.SequentialCommand;
import org.firstinspires.ftc.teamcode.path.HeadingOp;
import org.firstinspires.ftc.teamcode.path.HermiteSpline;
import org.firstinspires.ftc.teamcode.path.Movement;
import org.firstinspires.ftc.teamcode.path.TurnTo;
import org.firstinspires.ftc.teamcode.util.Pose;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/// Builds the Dribble command an auto describes.
public class PlaymakerLoader {

    public static final int VERSION = 1;

    private final Map<String, PlaymakerAction> actions;

    private final Map<String, PlaymakerNodeType> types = new LinkedHashMap<>();

    public PlaymakerLoader() {
        this(PlaymakerActions.discover());
    }

    public PlaymakerLoader(Map<String, PlaymakerAction> actions) {

        this.actions = actions;

        addType("sequential", node -> new SequentialCommand(node.children()));
        addType("parallel", node -> new ParallelCommand(node.children()));
        addType("race", node -> new RaceCommand(node.children()));
        addType("deadline", node -> deadline(node.children()));

        addType("delay", node -> new Delay(node.number("seconds")));
        addType("action", this::action);

        addType("path", node -> new FollowPath(movement(node)));
        addType("turn", node -> new FollowPath(movement(node)));
    }

    public PlaymakerLoader addType(String type, PlaymakerNodeType factory) {

        types.put(type, factory);

        return this;
    }

    public Map<String, PlaymakerAction> getActions() {
        return actions;
    }

    public static Command load(String data) {
        return new PlaymakerLoader().read(data);
    }

    public Command read(String data) {

        Map<String, Object> root = Json.object(Json.parse(data));

        if (!root.containsKey("version")) throw new IllegalArgumentException("the auto is missing version");

        double version = Json.number(root.get("version"));

        if (version > VERSION) {
            throw new IllegalArgumentException(
                    "This auto was saved by a newer Playmaker (version "
                    + (int) version + ", this one reads " + VERSION + ")"
            );
        }

        if (!root.containsKey("root")) throw new IllegalArgumentException("the auto is missing root");

        return node(Json.object(root.get("root")));
    }

    Command node(Map<String, Object> at) {

        PlaymakerNode node = new PlaymakerNode(this, at);

        String type = node.type();

        PlaymakerNodeType factory = types.get(type);

        if (factory == null) {
            throw new IllegalArgumentException(
                    "Playmaker does not know the node type " + type
                    + ", add it with addType. Known: " + types.keySet()
            );
        }

        return factory.build(node);
    }

    private Command deadline(Command[] all) {

        Command[] alongside = new Command[all.length - 1];
        System.arraycopy(all, 1, alongside, 0, alongside.length);

        return new DeadlineCommand(all[0], alongside);
    }

    private Command action(PlaymakerNode node) {

        String ref = node.string("ref");

        PlaymakerAction found = actions.get(ref);

        if (found == null) {
            throw new IllegalArgumentException(
                    "No action named " + ref
                    + ", is its subsystem annotated and registered with the scheduler? Known: " + actions.keySet()
            );
        }

        return found.make(arguments(found, node));
    }

    private Object[] arguments(PlaymakerAction action, PlaymakerNode node) {

        Class<?>[] wants = action.getParameterTypes();

        List<Object> given = node.has("args") ? node.list("args") : new ArrayList<>();

        if (given.size() != wants.length) {
            throw new IllegalArgumentException(action.getName() + " takes " + wants.length
                    + " argument(s), the auto supplies " + given.size());
        }

        Object[] out = new Object[wants.length];

        for (int i = 0; i < out.length; i++) out[i] = coerce(action.getName(), wants[i], given.get(i));

        return out;
    }

    private Object coerce(String where, Class<?> want, Object value) {

        if (want == double.class || want == Double.class) return Json.number(value);
        if (want == float.class || want == Float.class) return (float) Json.number(value);
        if (want == int.class || want == Integer.class) return (int) Json.number(value);
        if (want == long.class || want == Long.class) return (long) Json.number(value);
        if (want == boolean.class || want == Boolean.class) return Json.bool(value, false);
        if (want == String.class) return Json.string(value);

        throw new IllegalArgumentException(
                where + " takes a " + want.getSimpleName()
                + ", which Playmaker cannot write into an auto"
        );
    }

    private Movement movement(PlaymakerNode node) {

        if ("turn".equals(node.type())) return new TurnTo(node.number("heading"));

        List<Object> raw = node.list("points");

        if (raw.size() < 2) throw new IllegalArgumentException("A path needs at least 2 points");

        Pose[] points = new Pose[raw.size()];

        for (int i = 0; i < points.length; i++) {

            List<Object> pair = Json.array(raw.get(i));

            if (pair.size() != 2) throw new IllegalArgumentException("A path point is an [x, y] pair");

            points[i] = new Pose(Json.number(pair.get(0)), Json.number(pair.get(1)));
        }

        HermiteSpline spline = new HermiteSpline(points);

        spline.setReversed(node.bool("reversed", false));

        if (node.has("heading")) spline.setHeadingOp(headingOp(node.object("heading")));

        if (node.has("replan")) spline.setReplan(node.number("replan"));

        return spline;
    }

    private HeadingOp headingOp(Map<String, Object> at) {

        String type = Json.string(at.get("type"));

        switch (type) {

            case "constant":
                return HeadingOp.constantHeading(Json.number(at.get("value")));

            case "linear":
                return HeadingOp.linearHeading(Json.number(at.get("start")), Json.number(at.get("end")));

            case "reflex":
                return HeadingOp.linearHeadingReflex(Json.number(at.get("start")), Json.number(at.get("end")));

            case "exponential":
                return HeadingOp.exponentialHeading(Json.number(at.get("start")), Json.number(at.get("end")));

            default: throw new IllegalArgumentException("Playmaker does not know the heading type " + type);
        }
    }
}

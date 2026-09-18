package org.firstinspires.ftc.teamcode.playmaker;

import org.firstinspires.ftc.teamcode.dribble.Command;

import java.util.List;
import java.util.Map;

/// One node of a Playmaker auto
public class PlaymakerNode {

    private final PlaymakerLoader loader;
    private final Map<String, Object> fields;

    private Command[] children;

    PlaymakerNode(PlaymakerLoader loader, Map<String, Object> fields) {

        this.loader = loader;
        this.fields = fields;
    }

    public String type() {
        return string("type");
    }

    public boolean has(String key) {
        return fields.containsKey(key) && fields.get(key) != null;
    }

    public Object raw(String key) {

        if (!fields.containsKey(key)) throw missing(key);

        return fields.get(key);
    }

    public double number(String key) {
        return Json.number(raw(key));
    }

    public double number(String key, double fallback) {
        return has(key) ? Json.number(fields.get(key)) : fallback;
    }

    public String string(String key) {
        return Json.string(raw(key));
    }

    public String string(String key, String fallback) {
        return has(key) ? Json.string(fields.get(key)) : fallback;
    }

    public boolean bool(String key, boolean fallback) {
        return Json.bool(fields.get(key), fallback);
    }

    public List<Object> list(String key) {
        return Json.array(raw(key));
    }

    public Map<String, Object> object(String key) {
        return Json.object(raw(key));
    }

    public Command[] children() {

        if (children != null) return children;

        List<Object> raw = list("children");

        if (raw.isEmpty()) throw new IllegalArgumentException("An empty " + fields.get("type") + " node has nothing to run");

        children = new Command[raw.size()];

        for (int i = 0; i < children.length; i++) children[i] = loader.node(Json.object(raw.get(i)));

        return children;
    }

    private IllegalArgumentException missing(String key) {
        return new IllegalArgumentException("A " + fields.get("type") + " node is missing " + key);
    }
}

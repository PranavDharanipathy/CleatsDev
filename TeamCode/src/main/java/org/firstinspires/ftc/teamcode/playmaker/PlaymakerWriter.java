package org.firstinspires.ftc.teamcode.playmaker;

import java.util.List;
import java.util.Map;

/// Writes auto made in Playmaker both as JSON and as the java file that carries it.
public class PlaymakerWriter {

    private PlaymakerWriter() {}

    public static String json(Object value) {

        StringBuilder out = new StringBuilder();

        write(out, value, 0);

        return out.toString();
    }

    public static String javaFile(String packageName, String className, String superName, String autoName, String json) {

        StringBuilder out = new StringBuilder();

        out.append("package ").append(packageName).append(";\n\n");

        out.append("import com.qualcomm.robotcore.eventloop.opmode.Autonomous;\n\n");

        out.append("import org.firstinspires.ftc.teamcode.playmaker.Playmaker;\n");

        if ("PlaymakerOpMode".equals(superName)) {
            out.append("import org.firstinspires.ftc.teamcode.playmaker.PlaymakerOpMode;\n");
        }

        out.append("\n//written by Playmaker, edit it there\n");

        out.append("@Autonomous(name = \"").append(autoName).append("\", group = \"Playmaker\")\n");
        out.append("@Playmaker(name = \"").append(autoName).append("\")\n");

        out.append("public class ").append(className).append(" extends ").append(superName).append(" {\n\n");

        out.append("    public static final String DATA =\n");

        String[] lines = json.split("\n", -1);

        for (int i = 0; i < lines.length; i++) {

            out.append("            \"").append(escape(lines[i]));

            if (i < lines.length - 1) out.append("\\n\" +\n");
            else out.append("\";\n");
        }

        out.append("\n    @Override\n");
        out.append("    protected String data() {\n");
        out.append("        return DATA;\n");
        out.append("    }\n");

        out.append("}\n");

        return out.toString();
    }

    private static String escape(String line) {

        StringBuilder out = new StringBuilder();

        for (int i = 0; i < line.length(); i++) {

            char c = line.charAt(i);

            if (c == '\\' || c == '"') out.append('\\');

            out.append(c);
        }

        return out.toString();
    }

    private static void write(StringBuilder out, Object value, int depth) {

        if (value == null) {

            out.append("null");
            return;
        }

        if (value instanceof Map) {

            writeObject(out, Json.object(value), depth);
            return;
        }

        if (value instanceof List) {

            writeArray(out, Json.array(value), depth);
            return;
        }

        if (value instanceof Boolean) {

            out.append(value.toString());
            return;
        }

        if (value instanceof Number) {

            out.append(number(((Number) value).doubleValue()));
            return;
        }

        out.append('"').append(quote(value.toString())).append('"');
    }

    private static void writeObject(StringBuilder out, Map<String, Object> map, int depth) {

        if (map.isEmpty()) {

            out.append("{}");
            return;
        }

        out.append("{\n");

        int left = map.size();

        for (Map.Entry<String, Object> entry : map.entrySet()) {

            indent(out, depth + 1);

            out.append('"').append(quote(entry.getKey())).append("\": ");

            write(out, entry.getValue(), depth + 1);

            if (--left > 0) out.append(',');

            out.append('\n');
        }

        indent(out, depth);
        out.append('}');
    }

    private static void writeArray(StringBuilder out, List<Object> list, int depth) {

        if (list.isEmpty()) {

            out.append("[]");
            return;
        }

        if (flat(list)) {

            out.append('[');

            for (int i = 0; i < list.size(); i++) {

                if (i > 0) out.append(", ");

                write(out, list.get(i), depth);
            }

            out.append(']');
            return;
        }

        out.append("[\n");

        for (int i = 0; i < list.size(); i++) {

            indent(out, depth + 1);

            write(out, list.get(i), depth + 1);

            if (i < list.size() - 1) out.append(',');

            out.append('\n');
        }

        indent(out, depth);
        out.append(']');
    }

    private static boolean flat(List<Object> list) {

        for (Object value : list) {

            if (value instanceof Map) return false;

            if (value instanceof List && !flat(Json.array(value))) return false;
        }

        return true;
    }

    private static void indent(StringBuilder out, int depth) {

        for (int i = 0; i < depth; i++) {
            out.append("  ");
        }
    }

    private static String quote(String text) {

        StringBuilder out = new StringBuilder();

        for (int i = 0; i < text.length(); i++) {

            char c = text.charAt(i);

            switch (c) {

                case '"':  out.append("\\\""); break;
                case '\\': out.append("\\\\"); break;
                case '\b': out.append("\\b");  break;
                case '\f': out.append("\\f");  break;
                case '\n': out.append("\\n");  break;
                case '\r': out.append("\\r");  break;
                case '\t': out.append("\\t");  break;

                default:

                    if (c < 0x20) out.append("\\u").append(hex(c));
                    else out.append(c);
            }
        }

        return out.toString();
    }

    private static String hex(char c) {

        String digits = Integer.toHexString(c);

        StringBuilder out = new StringBuilder();

        for (int i = digits.length(); i < 4; i++) out.append('0');

        return out.append(digits).toString();
    }

    private static String number(double value) {

        if (value == Math.rint(value) && !Double.isInfinite(value) && Math.abs(value) < 1e15) {
            return Long.toString((long) value);
        }

        return Double.toString(value);
    }
}

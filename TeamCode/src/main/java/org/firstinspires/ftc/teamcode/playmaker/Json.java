package org.firstinspires.ftc.teamcode.playmaker;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class Json {

    private final String text;
    private int at;

    private Json(String text) {
        this.text = text;
    }

    public static Object parse(String text) {

        Json json = new Json(text);

        json.skipSpace();

        Object value = json.value();

        json.skipSpace();

        if (json.at < json.text.length()) throw json.fail("trailing text");

        return value;
    }

    @SuppressWarnings("unchecked")
    public static Map<String, Object> object(Object value) {

        if (!(value instanceof Map)) throw new IllegalArgumentException("Playmaker expected an object, got " + describe(value));

        return (Map<String, Object>) value;
    }

    @SuppressWarnings("unchecked")
    public static List<Object> array(Object value) {

        if (!(value instanceof List)) throw new IllegalArgumentException("Playmaker expected an array, got " + describe(value));

        return (List<Object>) value;
    }

    public static String string(Object value) {

        if (!(value instanceof String)) throw new IllegalArgumentException("Playmaker expected a string, got " + describe(value));

        return (String) value;
    }

    public static double number(Object value) {

        if (!(value instanceof Double)) throw new IllegalArgumentException("Playmaker expected a number, got " + describe(value));

        return (Double) value;
    }

    public static boolean bool(Object value, boolean fallback) {

        if (value == null) return fallback;

        if (!(value instanceof Boolean)) throw new IllegalArgumentException("Playmaker expected true or false, got " + describe(value));

        return (Boolean) value;
    }

    private static String describe(Object value) {

        if (value == null) return "null";

        return value.getClass().getSimpleName();
    }

    private Object value() {

        char c = peek();

        if (c == '{') return objectBody();
        if (c == '[') return arrayBody();
        if (c == '"') return stringBody();

        if (word("true")) return Boolean.TRUE;
        if (word("false")) return Boolean.FALSE;
        if (word("null")) return null;

        return numberBody();
    }

    private Map<String, Object> objectBody() {

        Map<String, Object> map = new LinkedHashMap<>();

        expect('{');
        skipSpace();

        if (peek() == '}') {

            at++;
            return map;
        }

        while (true) {

            skipSpace();

            String key = stringBody();

            skipSpace();
            expect(':');
            skipSpace();

            map.put(key, value());

            skipSpace();

            char c = next();

            if (c == '}') return map;
            if (c != ',') throw fail("expected , or } in an object");
        }
    }

    private List<Object> arrayBody() {

        List<Object> list = new ArrayList<>();

        expect('[');
        skipSpace();

        if (peek() == ']') {

            at++;
            return list;
        }

        while (true) {

            skipSpace();

            list.add(value());

            skipSpace();

            char c = next();

            if (c == ']') return list;
            if (c != ',') throw fail("expected , or ] in an array");
        }
    }

    private String stringBody() {

        expect('"');

        StringBuilder out = new StringBuilder();

        while (true) {

            char c = next();

            if (c == '"') return out.toString();

            if (c != '\\') {

                out.append(c);
                continue;
            }

            char escape = next();

            switch (escape) {

                case '"':  out.append('"');  break;
                case '\\': out.append('\\'); break;
                case '/':  out.append('/');  break;
                case 'b':  out.append('\b'); break;
                case 'f':  out.append('\f'); break;
                case 'n':  out.append('\n'); break;
                case 'r':  out.append('\r'); break;
                case 't':  out.append('\t'); break;

                case 'u':

                    if (at + 4 > text.length()) throw fail("truncated unicode escape");

                    out.append((char) Integer.parseInt(text.substring(at, at + 4), 16));
                    at += 4;
                    break;

                default: throw fail("unknown escape \\" + escape);
            }
        }
    }

    private Double numberBody() {

        int from = at;

        if (peek() == '-' || peek() == '+') at++;

        while (at < text.length()) {

            char c = text.charAt(at);

            if ((c >= '0' && c <= '9') || c == '.' || c == 'e' || c == 'E' || c == '-' || c == '+') at++;
            else break;
        }

        if (from == at) throw fail("expected a value");

        try {
            return Double.valueOf(text.substring(from, at));
        }
        catch (NumberFormatException e) {
            throw fail("bad number " + text.substring(from, at));
        }
    }

    private boolean word(String expected) {

        if (!text.startsWith(expected, at)) return false;

        at += expected.length();
        return true;
    }

    private void skipSpace() {

        while (at < text.length() && text.charAt(at) <= ' ') at++;
    }

    private char peek() {

        if (at >= text.length()) throw fail("ended early");

        return text.charAt(at);
    }

    private char next() {

        char c = peek();
        at++;

        return c;
    }

    private void expect(char c) {

        if (next() != c) throw fail("expected " + c);
    }

    private IllegalArgumentException fail(String why) {
        return new IllegalArgumentException("Playmaker json at " + at + ": " + why);
    }
}

package org.firstinspires.ftc.teamcode.playmaker;

import org.firstinspires.ftc.teamcode.dribble.Command;
import org.firstinspires.ftc.teamcode.dribble.CommandScheduler;
import org.firstinspires.ftc.teamcode.dribble.Subsystem;
import org.firstinspires.ftc.teamcode.playmaker.markers.PlaymakerIgnore;
import org.firstinspires.ftc.teamcode.playmaker.markers.PlaymakerSubsystem;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

/// Finds every action Playmaker can offer, from the subsystems registered with the scheduler.
public class PlaymakerActions {

    // Coding with John's Java annotation tutorial carried here bro.

    //Subsystem's own interface methods are not allowed
    private static final Set<String> SUBSYSTEM_METHODS = new HashSet<>(Arrays.asList("loop", "initialize", "getDefaultCommand", "getSubsystem"));

    private PlaymakerActions() {}

    public static Map<String, PlaymakerAction> discover() {
        return discover(CommandScheduler.getSubsystems());
    }

    public static Map<String, PlaymakerAction> discover(List<Subsystem> subsystems) {

        Map<String, PlaymakerAction> actions = new LinkedHashMap<>();

        for (Subsystem subsystem : subsystems) {

            Class<?> type = subsystem.getClass();

            PlaymakerSubsystem subMarker = type.getAnnotation(PlaymakerSubsystem.class);

            if (subMarker == null) continue;

            String prefix = subMarker.name().isEmpty() ? type.getSimpleName() : subMarker.name();

            Set<String> seen = new HashSet<>();

            for (Class<?> at = type; at != null && at != Object.class; at = at.getSuperclass()) {

                for (Method method : at.getDeclaredMethods()) {

                    if (!usable(method)) continue;

                    //an override shows up on the class and on its parent
                    if (!seen.add(signature(method))) continue;

                    String name = prefix + "." + method.getName();

                    actions.put(name, new PlaymakerAction(name, subsystem, method));
                }
            }
        }

        return Collections.unmodifiableMap(actions);
    }

    private static boolean usable(Method method) {

        if (method.isSynthetic() || method.isBridge()) return false;

        //kotlin internal mangles to name$module, lambdas to lambda$name$0
        if (method.getName().indexOf('$') >= 0) return false;

        if (!Modifier.isPublic(method.getModifiers())) return false;

        if (SUBSYSTEM_METHODS.contains(method.getName())) return false;

        if (method.isAnnotationPresent(PlaymakerIgnore.class)) return false;

        Class<?> returns = method.getReturnType();

        return returns == void.class || Command.class.isAssignableFrom(returns);
    }

    private static String signature(Method method) {
        return method.getName() + Arrays.toString(method.getParameterTypes());
    }
}

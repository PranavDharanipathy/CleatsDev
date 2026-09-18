package org.firstinspires.ftc.teamcode.playmaker;

import org.firstinspires.ftc.teamcode.dribble.Command;
import org.firstinspires.ftc.teamcode.dribble.InstantCommand;
import org.firstinspires.ftc.teamcode.dribble.Subsystem;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

/// One method on a subsystem that Playmaker can put on a timeline.
public class PlaymakerAction {

    private final String name;
    private final Subsystem subsystem;
    private final Method method;

    PlaymakerAction(String name, Subsystem subsystem, Method method) {

        this.name = name;
        this.subsystem = subsystem;
        this.method = method;

        method.setAccessible(true);
    }

    public String getName() {
        return name;
    }

    public Class<?>[] getParameterTypes() {
        return method.getParameterTypes();
    }

    public boolean isCommand() {
        return Command.class.isAssignableFrom(method.getReturnType());
    }

    /// A method that returns a Command is asked for it now, one that returns nothing is
    /// wrapped so it runs when the command does.
    public Command make(Object... arguments) {

        if (isCommand()) return (Command) invoke(arguments);

        return new InstantCommand(() -> invoke(arguments));
    }

    private Object invoke(Object... arguments) {

        try {
            return method.invoke(subsystem, arguments);
        }
        catch (IllegalAccessException e) {
            throw new IllegalStateException("Playmaker cannot reach " + name, e);
        }
        catch (InvocationTargetException e) {
            throw new IllegalStateException(name + " threw", e.getCause());
        }
    }

    @Override
    public String toString() {
        return name;
    }
}

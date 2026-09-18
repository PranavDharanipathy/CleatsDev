package org.firstinspires.ftc.teamcode.playmaker.markers;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/// Lets the command be usable in Playmaker
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface PlaymakerCommand {

    String name() default "";
}

package org.firstinspires.ftc.teamcode.util;

import java.util.function.Supplier;

public class Lazy<O> {

    private final Supplier<O> objSupplier;

    private O obj;

    public Lazy(Supplier<O> objSupplier) {
        this.objSupplier = objSupplier;
    }

    public O get() {

        if (obj == null) obj = objSupplier.get();

        return obj;
    }
}

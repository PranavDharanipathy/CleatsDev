package org.firstinspires.ftc.teamcode.playmaker;

import org.firstinspires.ftc.teamcode.dribble.DribbleOpMode;

/// Runs an auto written in Playmaker
public abstract class PlaymakerOpMode extends DribbleOpMode {

    protected abstract String data();

    protected void configure(PlaymakerLoader loader) {}

    @Override
    public void onStart() {

        PlaymakerLoader loader = new PlaymakerLoader();

        configure(loader);

        schedule(loader.read(data()));
    }
}

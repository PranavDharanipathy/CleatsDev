package org.firstinspires.ftc.teamcode.playmaker;

import org.firstinspires.ftc.teamcode.dribble.Command;

/// Builds a certain type of node.
/// <p>
/// Add your own with {@link PlaymakerLoader#addType}
@FunctionalInterface
public interface PlaymakerNodeType {

    Command build(PlaymakerNode node);
}

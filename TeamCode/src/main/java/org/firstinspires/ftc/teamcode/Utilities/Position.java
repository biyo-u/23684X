package org.firstinspires.ftc.teamcode.Utilities;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.jetbrains.annotations.NotNull;

public class Position {

    private Rotation rotation;
    private Vector2D vector2D;

    public Position(double x, double y, double heading, AngleUnit unit) {
        this.rotation = new Rotation(heading, unit);
        this.vector2D = new Vector2D(x, y);
    }

    public Rotation getRotation() {
        return rotation;
    }

    public void setRotation(Rotation rotation) {
        this.rotation = rotation;
    }

    public Vector2D getVector2D() {
        return vector2D;
    }

    public void setVector2D(Vector2D vector2D) {
        this.vector2D = vector2D;
    }

    public boolean equals(@NotNull Position obj) {
        return rotation.equals(obj.rotation) && vector2D.equals(obj.vector2D);
    }
}

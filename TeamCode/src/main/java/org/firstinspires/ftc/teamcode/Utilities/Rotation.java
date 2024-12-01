package org.firstinspires.ftc.teamcode.Utilities;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.jetbrains.annotations.NotNull;

public class Rotation {

    public double heading;

    public Rotation(double heading, AngleUnit unit) {
        if (unit == AngleUnit.RADIANS) {
            this.heading = Math.toDegrees(heading);
        } else {
            this.heading = heading;
        }
    }

    public double getHeading() {
        return heading;
    }

    public void setHeading(double heading, AngleUnit unit) {
        if (unit == AngleUnit.RADIANS) {
            this.heading = Math.toDegrees(heading);
        } else {
            this.heading = heading;
        }
    }

    public boolean equals(@NotNull Rotation obj) {
        return heading == obj.heading;
    }
}

package org.firstinspires.ftc.teamcode.Visual;

import android.graphics.drawable.GradientDrawable;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Common.Assembly;

public abstract class Visual extends Assembly {

    public abstract VectorF getPosition();
    public abstract Orientation getRotation();

    public enum STARTERSTACK {A,B,C}
    public abstract STARTERSTACK getStartStack();
    public abstract double getRingOffset();

    public abstract void update();
}

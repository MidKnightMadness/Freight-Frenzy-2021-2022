package org.firstinspires.ftc.teamcode.Visual;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Common.Assembly;

public abstract class Visual extends Assembly {

    public abstract VectorF getPosition();

    public enum STARTERSTACK {A,B,C}
    public abstract STARTERSTACK getStartStack();
}

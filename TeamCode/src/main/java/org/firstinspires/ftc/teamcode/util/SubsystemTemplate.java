package org.firstinspires.ftc.teamcode.util;

import com.seattlesolvers.solverslib.command.SubsystemBase;

public abstract class SubsystemTemplate extends SubsystemBase {
    public SubsystemTemplate initialize() { return this; }
    public abstract void init();
    public abstract void periodic();
}

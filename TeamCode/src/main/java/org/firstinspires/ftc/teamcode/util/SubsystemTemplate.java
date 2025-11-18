package org.firstinspires.ftc.teamcode.util;

import com.seattlesolvers.solverslib.command.SubsystemBase;

public abstract class SubsystemTemplate extends SubsystemBase {
    public SubsystemTemplate initialize() { return this; }
    public abstract void onAutonomousInit();
    public abstract void onTeleopInit();
    public abstract void onTeleopPeriodic();
    public abstract void onAutonomousPeriodic();
    public abstract void onTestPeriodic();
    public abstract void onTestInit();
    public abstract void onDisable();
}

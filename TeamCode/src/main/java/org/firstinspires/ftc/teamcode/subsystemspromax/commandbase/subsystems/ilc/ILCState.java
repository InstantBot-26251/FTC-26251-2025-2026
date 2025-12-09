package org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.ilc;

public enum ILCState {
    IDLE,           // Not doing anything
    SPINNING_UP,    // Flywheels spinning to target velocity
    READY,          // At target velocity, waiting for shoot command
    SHOOTING        // Gate open, transfer running
}

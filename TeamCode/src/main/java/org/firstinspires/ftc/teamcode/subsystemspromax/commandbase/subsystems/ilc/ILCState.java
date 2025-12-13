package org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.ilc;

public enum ILCState {
    IDLE,           // Not shooting, motors off
    REVERSING,      // Transfer running in reverse briefly
    SPINNING_UP,    // Flywheel spinning up to target velocity
    READY,          // At target velocity, ready to shoot
    SHOOTING,       // Actively shooting, transfer running forward
    MANUAL_FIRE     // For manual teleop shooting
}

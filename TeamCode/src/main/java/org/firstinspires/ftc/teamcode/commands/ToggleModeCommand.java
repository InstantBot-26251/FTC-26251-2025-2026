//package org.firstinspires.ftc.teamcode.commands;
//
//import com.seattlesolvers.solverslib.command.InstantCommand;
//import org.firstinspires.ftc.teamcode.subsystems.TRIkicker.KickerSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.TRIkicker.enums.OperatingMode;
//
//public class ToggleModeCommand extends InstantCommand {
//
//    private final KickerSubsystem kicker;
//
//    public ToggleModeCommand(KickerSubsystem kicker) {
//        this.kicker = kicker;
//        addRequirements(kicker);
//    }
//
//    @Override
//    public void initialize() {
//        if (kicker.getMode() == OperatingMode.TELEOP) {
//            kicker.setMode(OperatingMode.AUTO);
//        } else {
//            kicker.setMode(OperatingMode.TELEOP);
//        }
//    }
//}

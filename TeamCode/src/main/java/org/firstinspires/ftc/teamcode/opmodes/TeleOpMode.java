package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.util.GamePeriod;

@TeleOp (name = "YOU BETTER WIN")
public class TeleOpMode extends CommandOpMode {
    public Gamepad driver;
    public Gamepad operator;


    GamePeriod gamePeriod;

    TelemetryData telemetryData = new TelemetryData(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));

//    private final Enigma robot = Enigma.getInstance();


    @Override
    public void initialize() {
        gamePeriod = GamePeriod.TELE_OP;

        super.reset(); // Resets Command Scheduler


//        robot.teleopInit(telemetry, hardwareMap, driver, operator);
    }

    @Override
    public void run() {
//        robot.periodic();
    }

}

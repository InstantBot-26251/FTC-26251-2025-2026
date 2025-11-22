package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.globals.Enigma;
import org.firstinspires.ftc.teamcode.util.Alliance;

@TeleOp(name = "Test: Auto Aim", group = "Test")
public class AutoAimTest extends OpMode {

    private DriveSubsystem drive;
    private Enigma robot;

    public enum StateMachine {
        CLOSE,
        FAR,
        NO_LOCK
    }

    StateMachine state;

    @Override
    public void init() {
        Enigma robot = Enigma.getInstance();
        robot.init(hardwareMap, Alliance.BLUE);

        drive = DriveSubsystem.getInstance();

        drive.onTeleopInit();
    }

    @Override
    public void loop() {

        if (gamepad1.a) {
            drive.autoAimHeadingCLOSE();
            setState(StateMachine.CLOSE);
        }
        if (gamepad1.b) {
            drive.autoAimHeadingFAR();
            setState(StateMachine.FAR);
        }
        else {
            setState(StateMachine.NO_LOCK);
        }

        telemetry.addData("Auto Aim: ", getState());
    }

    public StateMachine getState() {
        return state;
    }

    public void setState(StateMachine state) {
        this.state = state;
    }
}

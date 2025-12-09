package org.firstinspires.ftc.teamcode.solverslib.opmodes.test.drive;

import static org.firstinspires.ftc.teamcode.solverslib.globals.Robot.DRIVE_SENSITIVITY;
import static org.firstinspires.ftc.teamcode.solverslib.globals.Robot.ROTATIONAL_SENSITIVITY;
import static org.firstinspires.ftc.teamcode.solverslib.globals.Robot.ROTATION_DAMPEN;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.solverslib.commandbase.commands.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.solverslib.globals.Robot;

@TeleOp(name = "Test: Auto Aim", group = "Test")
public class AutoAimTest extends OpMode {
    private Robot robot;

    public enum StateMachine {
        CLOSE,
        FAR,
        NO_LOCK
    }

    StateMachine state;

    @Override
    public void init() {
        robot = Robot.getInstance();
        robot.teleopInit(telemetry, hardwareMap);

    }

    @Override
    public void loop() {
        double leftY = gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x;

        new TeleopDriveCommand(
                () -> robot.applyResponseCurve(leftY, DRIVE_SENSITIVITY),
                () -> -robot.applyResponseCurve(leftX, DRIVE_SENSITIVITY),
                () -> -robot.applyResponseCurve(rightX, ROTATIONAL_SENSITIVITY) * ROTATION_DAMPEN);



        if (gamepad1.a) {
            robot.drive.autoAimHeadingCLOSE();
            setState(StateMachine.CLOSE);
        }
        if (gamepad1.b) {
            robot.drive.autoAimHeadingFAR();
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

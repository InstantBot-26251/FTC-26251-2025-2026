package org.firstinspires.ftc.teamcode.opmodes.test.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test: Drive", group = "Test")
public class DriveTest extends OpMode {

    Chassis chassis;

    @Override
    public void init() {
       chassis = new Chassis(hardwareMap);
    }

    @Override
    public void loop() {

        if (gamepad1.a) {
            chassis.fl.setPower(1);
            chassis.fr.setPower(1);
            chassis.bl.setPower(1);
            chassis.br.setPower(1);
        }

        if (gamepad1.dpad_up) {
            chassis.fl.setPower(0.5);  // Test front left
        } else if (gamepad1.dpad_right) {
            chassis.fr.setPower(0.5);  // Test front right
        } else if (gamepad1.dpad_down) {
            chassis.bl.setPower(0.5);  // Test back left
        } else if (gamepad1.dpad_left) {
            chassis.br.setPower(0.5);  // Test back right
        }


    }

}

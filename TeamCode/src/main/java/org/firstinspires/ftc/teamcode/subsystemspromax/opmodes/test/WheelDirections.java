package org.firstinspires.ftc.teamcode.subsystemspromax.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class WheelDirections extends OpMode {
    private DcMotorEx fl, fr, bl, br;

    @Override
    public void init() {
        fr = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.rightFrontMotorName);
        fl = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.leftFrontMotorName);
        bl = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.leftRearMotorName);
        br = hardwareMap.get(DcMotorEx.class, Constants.driveConstants.rightRearMotorName);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            fr.setPower(0.25);
            fl.setPower(0.25);
            bl.setPower(0.25);
            br.setPower(0.25);
        }
        if (gamepad1.right_bumper) {
            fr.setPower(0);
            fl.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }
    }
}

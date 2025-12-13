package org.firstinspires.ftc.teamcode.chucnotworkingsimple;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Intake {
    private DcMotorEx intake;

    private static final double INTAKE_POWER = 1.0;
    private static final double REVERSE_POWER = -1.0;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void run() {
        intake.setPower(INTAKE_POWER);
    }

    public void stop() {
        intake.setPower(0);
    }

    public void reverse() {
        intake.setPower(REVERSE_POWER);
    }

    public boolean isOverCurrent() {
        return intake.isOverCurrent();
    }
}
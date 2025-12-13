package org.firstinspires.ftc.teamcode.subsystemspromax.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.subsystemspromax.RobotE;
import org.firstinspires.ftc.teamcode.util.Alliance;

@TeleOp(name = "TeleOp")
public class TeleOpMode extends OpMode {
    RobotE robot;

    @Override
    public void init() {
        robot = new RobotE(hardwareMap, Alliance.BLUE);

        robot.onTeleopInit(telemetry, hardwareMap, gamepad1, gamepad2);
        robot.bindControls();
    }

    @Override
    public void init_loop() {
        if (gamepad1.aWasPressed()) {
            robot.alliance = Alliance.BLUE;
        }

        if (gamepad1.bWasPressed()) {
            robot.alliance = Alliance.RED;
        }

        robot.telemetry.addData("Alliance", robot.alliance);
        robot.telemetry.update();
    }

    @Override
    public void loop() {
        robot.periodic();
        robot.handleTeleopControls();

        robot.telemetry.addData("Follower Pose", robot.follower.getPose().toString());
        robot.telemetry.addData("ILC Velocity", robot.ilc.getVelocity());
        robot.telemetry.addData("ILC Target", robot.ilc.getTarget());
        robot.telemetry.addData("Hold Position", robot.headingLockEnabled);
        robot.telemetry.update();
    }

}

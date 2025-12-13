//package org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.commands;
//
//import com.pedropathing.ivy.Command;
//import com.pedropathing.util.Timer;
//
//import org.firstinspires.ftc.teamcode.subsystemspromax.RobotE;
//import org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.intake.IntakeState;
//
//public class Shoot extends Command {
//    private final RobotE robot;
//    private final Timer timer = new Timer();
//
//    private static final double TRANSFER_RUN_TIME = 1.0; // Time to run transfer in seconds
//
//    private enum ShootState {
//        SPINNING_UP,
//        SHOOTING,
//        DONE
//    }
//
//    private ShootState state;
//
//    public Shoot(RobotE robot) {
//        this.robot = robot;
//    }
//
//    @Override
//    public void start() {
//        // Start spinning up the ILC
//        robot.ilc.startSpinup();
//        state = ShootState.SPINNING_UP;
//        timer.resetTimer();
//    }
//
//    @Override
//    public void execute() {
//        switch (state) {
//            case SPINNING_UP:
//                // Wait for ILC to be ready
//                if (robot.ilc.isReady()) {
//                    // Open gate and start transfer
//                    robot.ilc.shoot();
//                    robot.intake.setIntake(IntakeState.TRANSFERRING);
//                    timer.resetTimer();
//                    state = ShootState.SHOOTING;
//                }
//                break;
//
//            case SHOOTING:
//                // Wait for transfer to complete
//                if (timer.getElapsedTimeSeconds() > TRANSFER_RUN_TIME) {
//                    // Stop everything
//                    robot.ilc.stopShooting();
//                    robot.intake.setIDLE();
//                    state = ShootState.DONE;
//                }
//                break;
//
//            case DONE:
//                // Command will end on next isFinished() check
//                break;
//        }
//    }
//
//    public boolean isFinished() {
//        return state == ShootState.DONE;
//    }
//
//    @Override
//    public void end(boolean interrupted) {
//        if (interrupted) {
//            // If interrupted, make sure everything is stopped
//            robot.ilc.stopShooting();
//            robot.intake.setIDLE();
//        }
//    }
//}
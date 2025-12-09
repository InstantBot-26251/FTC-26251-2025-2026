package org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.ilc;

import static org.firstinspires.ftc.teamcode.solverslib.commandbase.subsystems.vision.VisionConstants.arducam_cx;
import static org.firstinspires.ftc.teamcode.solverslib.commandbase.subsystems.vision.VisionConstants.arducam_cy;
import static org.firstinspires.ftc.teamcode.solverslib.commandbase.subsystems.vision.VisionConstants.arducam_fx;
import static org.firstinspires.ftc.teamcode.solverslib.commandbase.subsystems.vision.VisionConstants.arducam_fy;
import static org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.ilc.ILCConstants.*;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.solverslib.globals.RobotMap;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@Config
public class InertialLaunchCore {
    // Vision
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Gate servo
    private Servo gate;
    // ilcAlpha is left, ilcBeta is right
    private DcMotorEx ilcAlpha, ilcBeta;

    public int shotsRemaining = 0;

    // State variables
    private double target = 0;
    private boolean isILCActivated = true;


    // Class variables
    private ILCState ilcState = ILCState.IDLE;
    private ElapsedTime stateTimer = new ElapsedTime();
    private double targetVelocity = 0;


    public InertialLaunchCore(HardwareMap hardwareMap) {
        makeProcessor();
        makePortal();

        ilcAlpha = hardwareMap.get(DcMotorEx.class, "ilcL"); // ILC dual motors
        ilcBeta = hardwareMap.get(DcMotorEx.class, "ilcR");
        gate = hardwareMap.get(Servo.class, "gate"); // Gate servo

        ilcAlpha.setDirection(DcMotorSimple.Direction.REVERSE);

        ilcState = ILCState.IDLE;

        setPower(0);
        gateStop();
    }

    // PERIODIC UPDATE
    public void periodic() {
        update();
        if (isILCActivated) {
            updateVelocityPID();
        }
    }

    // STATE MACHINE UPDATE
    public void update() {
        switch (ilcState) {
            case IDLE:
                // waiting for startSpinup() command
                break;

            case SPINNING_UP:
                // Check if at target velocity
                if (isAtTarget()) {
                    ilcState = ILCState.READY;
                }

                // Timeout safety
                if (stateTimer.seconds() > FLYWHEEL_MAX_SPINUP_TIME) {
                    ilcState = ILCState.READY; // Ready even if not perfect
                }
                break;

            case READY:
                // Maintaining velocity, waiting for shoot() command
                // Could add safety timeout here to spin down if no shot
                break;

            case SHOOTING:
                // Gate is open, transfer should be running
                // Auto will call stopShooting() when transfer is done
                break;
        }
    }

    // GETTERS (for state)
    public double getTarget() {
        return target;
    }

    public double getVelocity() {
        return ilcBeta.getVelocity();
    }

    public boolean isILCActivated() {
        return isILCActivated;
    }

    public boolean isAtTarget() {
        return Math.abs((getTarget() - getVelocity())) < 50;
    }

    public boolean isAtShootPosition() {
        return gate.getPosition() == SHOOT_POSITION;
    }

    // SETTERS
    public void setPower(double power) {
        ilcAlpha.setPower(power);
        ilcBeta.setPower(power);
    }

    public void setTarget(double velocity) {
        target = velocity;
    }

    // AUTO
    // COMMANDS FOR AUTO
    public void startSpinup() {
        if (ilcState == ILCState.IDLE) {
            double distance = getDistance();
            targetVelocity = interpolateRPM(distance);
            setTarget(targetVelocity);
            activateILC();
            stateTimer.reset();
            ilcState = ILCState.SPINNING_UP;
        }
    }

    public void startSpinupWithDistance(double distance) {
        if (ilcState == ILCState.IDLE) {
            targetVelocity = interpolateRPM(distance);
            setTarget(targetVelocity);
            activateILC();
            stateTimer.reset();
            ilcState = ILCState.SPINNING_UP;
        }
    }

    // Update velocity during spinup if robot is moving
    public void updateVelocityDuringSpinup() {
        if (ilcState == ILCState.SPINNING_UP) {
            double distance = getDistance();
            if (distance > 0) {
                targetVelocity = interpolateRPM(distance);
                setTarget(targetVelocity);
            }
        }
    }

    public void shoot() {
        if (ilcState == ILCState.READY) {
            gateLaunch();
            stateTimer.reset();
            ilcState = ILCState.SHOOTING;
        }
    }

    public void stopShooting() {
        if (ilcState == ILCState.SHOOTING) {
            gateStop();
            deactivateILC();
            ilcState = ILCState.IDLE;
        }
    }

    public void forceIdle() {
        deactivateILC();
        gateStop();
        ilcState = ILCState.IDLE;
    }

    // QUERY METHODS
    public boolean isReady() {
        return ilcState == ILCState.READY;
    }

    public boolean isShooting() {
        return ilcState == ILCState.SHOOTING;
    }

    public boolean isIdle() {
        return ilcState == ILCState.IDLE;
    }

    public boolean isSpinningUp() {
        return ilcState == ILCState.SPINNING_UP;
    }

    public ILCState getState() {
        return ilcState;
    }

    // CONTROL METHODS
    public void updateVelocityPID() {
        setPower((kV * getTarget()) + (kP * (getTarget() - getVelocity())) + kS);
    }

    public void gateLaunch() {
        gate.setPosition(SHOOT_POSITION);
    }

    public void gateStop() {
        gate.setPosition(STOP_POSITION);
    }

    public void flipGate() {
        if (gate.getPosition() == STOP_POSITION)
            gateLaunch();
        else
            gateStop();
    }

    public void activateILC() {
        isILCActivated = true;
    }

    public void deactivateILC() {
        isILCActivated = false;
        setPower(0);
    }

    public void setVelocityForDistance(double distance) {
        double requiredRPM = interpolateRPM(distance);
        setTarget(requiredRPM);
    }

    public void setVelocityByDistance() {
        double requiredRPM = interpolateRPM(getDistance());
        setTarget(requiredRPM);
    }

    /**
     * LinEaR InTeRPolATiOn
     * If distance isn't a valid one, it will clamp to the nearest point
     *
     * @param distance Target distance - measure in inches
     * @return Interpolated RPM value
     */
    private double interpolateRPM(double distance) {
        // Handle edge cases
        if (distanceRPMLUT.length == 0) {
            return 0;
        }

        if (distance <= distanceRPMLUT[0][0]) {
            return distanceRPMLUT[0][1];
        }

        if (distance >= distanceRPMLUT[distanceRPMLUT.length - 1][0]) {
            return distanceRPMLUT[distanceRPMLUT.length - 1][1];
        }

        // Find the two points to interpolate between
        for (int i = 0; i < distanceRPMLUT.length - 1; i++) {
            double d1 = distanceRPMLUT[i][0];
            double rpm1 = distanceRPMLUT[i][1];
            double d2 = distanceRPMLUT[i + 1][0];
            double rpm2 = distanceRPMLUT[i + 1][1];

            if (distance >= d1 && distance <= d2) {
                // Linear interpolation formula: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
                double t = (distance - d1) / (d2 - d1);
                return rpm1 + t * (rpm2 - rpm1);
            }
        }

        // Fallback (should never reach here)
        return distanceRPMLUT[0][1];
    }

    // VISION
    public void makeProcessor() {
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(arducam_fx, arducam_fy, arducam_cx, arducam_cy)
                .build();
    }

    public void makePortal() {
        VisionPortal.Builder portalBuilder = new VisionPortal.Builder()
                .setCamera(RobotMap.getInstance().ARDUCAM)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setAutoStopLiveView(true)
                .addProcessor(aprilTag);

        visionPortal = portalBuilder.build();
    }
    public ArrayList<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    public int getID() {
        ArrayList<AprilTagDetection> detections = getDetections();

        if (!detections.isEmpty()) {
            return detections.get(0).id; // get the ID of the first detected tag
        } else {
            return -1; // return -1 (or some other value) if no tags are detected
        }
    }

    public double getDistance() {
        ArrayList<AprilTagDetection> detections = getDetections();

        if (detections != null && !detections.isEmpty()) {
            AprilTagDetection detection = detections.get(0);
            if (detection.ftcPose != null) {
                return detection.ftcPose.range;
            }
        }

        return -1.0;
    }

    public String getMotif() {
        ArrayList<AprilTagDetection> detections = getDetections();

        if (detections != null && !detections.isEmpty()) {
            AprilTagDetection best = detections.get(0);

            if (best != null && best.metadata != null && best.metadata.name != null) {
                return best.metadata.name;   // e.g. "PGP"
            }
        }

        return "UNKNOWN";
    }

    public void stopStreaming() {
        visionPortal.stopStreaming();
    }

    public void resumeStreaming() {
        visionPortal.resumeStreaming();
    }

    public void closePortal() {
        visionPortal.close();
    }

}
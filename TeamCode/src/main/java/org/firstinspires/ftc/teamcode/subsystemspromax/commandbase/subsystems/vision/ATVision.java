//package org.firstinspires.ftc.teamcode.subsystemspromax.commandbase.subsystems.vision;
//
//import static org.firstinspires.ftc.teamcode.solverslib.commandbase.subsystems.vision.VisionConstants.arducam_cx;
//import static org.firstinspires.ftc.teamcode.solverslib.commandbase.subsystems.vision.VisionConstants.arducam_cy;
//import static org.firstinspires.ftc.teamcode.solverslib.commandbase.subsystems.vision.VisionConstants.arducam_fx;
//import static org.firstinspires.ftc.teamcode.solverslib.commandbase.subsystems.vision.VisionConstants.arducam_fy;
//
//import android.util.Size;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.solverslib.globals.RobotMap;
//
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//
//import java.util.ArrayList;
//
//public class ATVision {
//    private Telemetry telemetry;
//    private AprilTagProcessor aprilTag;
//    private VisionPortal visionPortal;
//
//    private static final ATVision INSTANCE = new ATVision();
//
//    public static ATVision getInstance() {
//        return INSTANCE;
//    }
//
////    @Override
////    public void init() {
////        makeProcessor();
////        makePortal();
////    }
//
////    @Override
////    public void periodic() {
////        updateTelemetry();
////    }
//
//    public void makeProcessor() {
//        aprilTag = new AprilTagProcessor.Builder()
//                .setLensIntrinsics(arducam_fx, arducam_fy, arducam_cx, arducam_cy)
//                .build();
//    }
//
//    public void makePortal() {
//        VisionPortal.Builder portalBuilder = new VisionPortal.Builder()
//                .setCamera(RobotMap.getInstance().ARDUCAM)
//                .setCameraResolution(new Size(640, 480))
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//                .setAutoStopLiveView(true)
//                .addProcessor(aprilTag);
//
//        visionPortal = portalBuilder.build();
//    }
//
//    public ArrayList<AprilTagDetection> getDetections() {
//        return aprilTag.getDetections();
//    }
//
//    public int getID() {
//        ArrayList<AprilTagDetection> detections = getDetections();
//
//        if (!detections.isEmpty()) {
//            return detections.get(0).id; // get the ID of the first detected tag
//        } else {
//            return -1; // return -1 (or some other value) if no tags are detected
//        }
//    }
//
//    public double getDistance() {
//        ArrayList<AprilTagDetection> detections = getDetections();
//
//        if (detections != null && !detections.isEmpty()) {
//            AprilTagDetection detection = detections.get(0);
//            if (detection.ftcPose != null) {
//                return detection.ftcPose.range;
//            }
//        }
//
//        return -1.0;
//    }
//
//    public String getMotif() {
//        ArrayList<AprilTagDetection> detections = getDetections();
//
//        if (detections != null && !detections.isEmpty()) {
//            AprilTagDetection best = detections.get(0);
//
//            if (best != null && best.metadata != null && best.metadata.name != null) {
//                return best.metadata.name;   // e.g. "PGP"
//            }
//        }
//
//        return "UNKNOWN";
//    }
//
//    public void stopStreaming() {
//        visionPortal.stopStreaming();
//    }
//
//    public void resumeStreaming() {
//        visionPortal.resumeStreaming();
//    }
//
//    public void closePortal() {
//        visionPortal.close();
//    }
//
//    private void updateTelemetry() {
//        if (telemetry == null) return;
//
//        ArrayList<AprilTagDetection> detections = getDetections();
//
//        if (detections != null && !detections.isEmpty()) {
//            AprilTagDetection detection = detections.get(0);
//            telemetry.addData("AprilTag ID", detection.id);
//            telemetry.addData("Motif", getMotif());
//
//            if (detection.ftcPose != null) {
//                telemetry.addData("Distance", "%.1f inches", detection.ftcPose.range);
//                telemetry.addData("Bearing", "%.1f deg", detection.ftcPose.bearing);
//                telemetry.addData("Elevation", "%.1f deg", detection.ftcPose.elevation);
//            }
//        } else {
//            telemetry.addData("AprilTag", "None detected");
//        }
//    }
//}
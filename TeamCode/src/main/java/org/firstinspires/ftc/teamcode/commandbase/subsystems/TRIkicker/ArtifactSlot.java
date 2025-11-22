//package org.firstinspires.ftc.teamcode.subsystems.TRIkicker;
//
//import org.firstinspires.ftc.teamcode.subsystems.TRIkicker.enums.ArtifactColor;
//
//public class ArtifactSlot {
//
//    private ArtifactColor color = ArtifactColor.UNKNOWN;
//    private boolean hasConfirmedArtifact = false;
//    private int detectionConfidence = 0; // 0 = no detection, 1 = one sensor, 2 = both sensors
//
//    public ArtifactSlot() {
//        clear();
//    }
//
//    public void setColor(ArtifactColor color) {
//        this.color = color;
//    }
//
//    public ArtifactColor getColor() {
//        return color;
//    }
//
//    public void setHasConfirmedArtifact(boolean hasArtifact) {
//        this.hasConfirmedArtifact = hasArtifact;
//    }
//
//    public boolean hasConfirmedArtifact() {
//        return hasConfirmedArtifact;
//    }
//
//    public void setDetectionConfidence(int confidence) {
//        this.detectionConfidence = confidence;
//    }
//
//    public int getDetectionConfidence() {
//        return detectionConfidence;
//    }
//
//    public void clear() {
//        this.color = ArtifactColor.UNKNOWN;
//        this.hasConfirmedArtifact = false;
//        this.detectionConfidence = 0;
//    }
//
//    @Override
//    public String toString() {
//        if (!hasConfirmedArtifact) {
//            return "EMPTY";
//        }
//        String confidenceStr = detectionConfidence == 2 ? "BOTH" : "ONE";
//        return color + " (" + confidenceStr + ")";
//    }
//}

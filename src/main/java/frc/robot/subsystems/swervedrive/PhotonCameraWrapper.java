package frc.robot.subsystems.swervedrive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

import java.io.IOException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonCameraWrapper {
  private final PhotonCamera camera = new PhotonCamera("Global_Shutter_Camera");
  private AprilTagFieldLayout aprilTagFieldLayout;
  private final PhotonPoseEstimator photonPoseEstimator;
  private double lastEstTimestamp = 0;

  /** Create a new PhotonCameraWrapper to interface with photonvision camera. */
  public PhotonCameraWrapper() {
    aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    aprilTagFieldLayout.setOrigin(
        DriverStation.getAlliance().get() == Alliance.Blue
            ? OriginPosition.kBlueAllianceWallRightSide
            : OriginPosition.kRedAllianceWallRightSide);

    var robotToCam = new Transform3d(Constants.cameraTranslation, Constants.cameraRotation);
    photonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);
    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public void setAlliance() {
    aprilTagFieldLayout.setOrigin(
        DriverStation.getAlliance().get() == Alliance.Blue
            ? OriginPosition.kBlueAllianceWallRightSide
            : OriginPosition.kRedAllianceWallRightSide);
    photonPoseEstimator.setFieldTags(aprilTagFieldLayout);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var visionEst = photonPoseEstimator.update();
    double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    
    return photonPoseEstimator.update();
  }

  public PhotonPipelineResult getLatestResult() {
    // TODO Auto-generated method stub
  return camera.getLatestResult();
  }
}

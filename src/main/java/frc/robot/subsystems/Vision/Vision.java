package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap.VisionConfig;
import frc.robot.subsystems.Vision.LimelightHelpers.LimelightTarget_Fiducial;
import java.text.DecimalFormat;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/*
 * This class requires MAJOR CLEANUP. There needs to be a proper pyramid of hierarchy. Vision should NOT be able to control anything related to pose. It should only
 * broadcast its current pose, if it has one, for use by the Pose class. Vision --> Pose. Vision should NEVER be able to control robot odometry.
 *
 */

public class Vision extends SubsystemBase {
  private Pose2d botPose;
  private Pose2d tempPose;
  private double limeLatency;
  private boolean apriltagLimelightConnected = false;
  private boolean NNLimelightConnected = false;

  private double photonTimestamp;
  private PhotonCamera photonCam_1;
  private boolean photon1HasTargets;
  private AprilTagFieldLayout aprilTagFieldLayout;
  private PhotonPoseEstimator photonPoseEstimator;
  private Transform3d robotToCam;

  private PhotonCamera photonCam_2;
  private boolean photon2HasTargets;
  private PhotonPoseEstimator photonPoseEstimator_2;
  private Transform3d robotToCam_2;

  // Shuffleboard telemetry - pose estimation
  private ShuffleboardTab tab = Shuffleboard.getTab("Vision");
  private GenericEntry visionXDataEntry = tab.add("VisionPose X", 0).getEntry();
  private GenericEntry visionYDataEntry = tab.add("VisionPose Y", 0).getEntry();
  private GenericEntry visionRotDataEntry = tab.add("VisionPose Rotation", 0).getEntry();

  // Shuffleboard telemtry - note detection
  private GenericEntry visionNotePoseRobRelXEntry = tab.add("NoteX Pose Robot Space", 0).getEntry();
  private GenericEntry visionNotePoseRobRelYEntry = tab.add("NoteY Pose Robot Space", 0).getEntry();
  private GenericEntry visionNotePoseRobRelRotEntry = tab.add("Note Rotation Pose Robot Space", 0).getEntry();

  private GenericEntry visionNotePoseFieldRelXEntry = tab.add("NoteX Pose Field Space", 0).getEntry();
  private GenericEntry visionNotePoseFieldRelYEntry = tab.add("NoteY Pose Field Space", 0).getEntry();
  private GenericEntry visionNotePoseFieldRelRotEntry = tab.add("Note Rotation Pose Field Space", 0).getEntry();

  // For Note detection in the future
  private double detectHorizontalOffset = 0;
  private double detectVerticalOffset = 0;

  private int targetSeenCount = 0;

  private boolean aimTarget = false;
  private boolean detectTarget = false;
  private LimelightHelpers.LimelightResults jsonResults, detectJsonResults;
  private Pose2d targetRobotRelativePose;
  private Pose2d noteFieldRelativePose;

  // testing
  private final DecimalFormat df = new DecimalFormat();

  private static Vision instance;

  public static synchronized Vision getInstance() {
    if (instance == null)
      instance = new Vision();
    return instance;
  }

  // TODO - see if adding setCameraPose_RobotSpace() is needed from
  // LimelightHelpers
  private Vision() {
    setName("Vision");
    botPose = new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(0)));
    tempPose = new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(0)));
    noteFieldRelativePose = new Pose2d();
    targetRobotRelativePose = new Pose2d();
    photonTimestamp = 0.0;
    limeLatency = 0.0;
    // botPose3d = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
    // targetSeenCount = 0;
    // aimHorizontalOffset = 0;
    // aimVerticalOffset = 0;

    // Changes vision mode between limelight and photonvision for easy switching
    if (VisionConfig.IS_LIMELIGHT_MODE) {
      // configure both limelights
      LimelightHelpers.setLEDMode_ForceOff(VisionConfig.POSE_LIMELIGHT);
      setLimelightPipeline(VisionConfig.POSE_LIMELIGHT, VisionConfig.APRILTAG_PIPELINE);
      LimelightHelpers.setCameraPose_RobotSpace(
          VisionConfig.POSE_LIMELIGHT,
          VisionConfig.POSE_LIME_X,
          VisionConfig.POSE_LIME_Y,
          VisionConfig.POSE_LIME_Z,
          VisionConfig.POSE_LIME_ROLL,
          VisionConfig.POSE_LIME_PITCH,
          VisionConfig.POSE_LIME_YAW);

      if (VisionConfig.IS_NEURAL_NET) {
        LimelightHelpers.setLEDMode_ForceOff(VisionConfig.NN_LIMELIGHT);
        setLimelightPipeline(VisionConfig.NN_LIMELIGHT, VisionConfig.NOTE_DETECTOR_PIPELINE);
      }
    }
    if (VisionConfig.IS_PHOTON_VISION_MODE) { // Configure photonvision camera
      photonCam_1 = new PhotonCamera(VisionConfig.POSE_PHOTON_1);
      // photonCam_2 = new PhotonCamera(VisionConfig.POSE_PHOTON_2);
      photon1HasTargets = false;
      try {
        aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      } catch (Exception e) {
        System.out.println("Field layout not found");
      }

      // Mounting information of photoncamera for making PhotonPoseEstimator object
      robotToCam = new Transform3d(
          new Translation3d(VisionConfig.CAM_1_X, VisionConfig.CAM_1_Y, VisionConfig.CAM_1_Z),
          new Rotation3d(
              VisionConfig.CAM_1_ROLL_RADIANS,
              VisionConfig.CAM_1_PITCH_RADIANS,
              VisionConfig.CAM_1_YAW_RADIANS));
      // TODO for 9th graders - create PhotonPoseEstimator object
      photonPoseEstimator = new PhotonPoseEstimator(
          aprilTagFieldLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          photonCam_1,
          robotToCam);

      // Mounting information of photoncamera for making PhotonPoseEstimator object
      // robotToCam_2 =
      // new Transform3d(
      // new Translation3d(VisionConfig.CAM_2_X, VisionConfig.CAM_2_Y,
      // VisionConfig.CAM_2_Z),
      // new Rotation3d(
      // VisionConfig.CAM_2_ROLL_RADIANS,
      // VisionConfig.CAM_2_PITCH_RADIANS,
      // VisionConfig.CAM_2_YAW_RADIANS));
      // // TODO for 9th graders - create PhotonPoseEstimator object
      // photonPoseEstimator_2 =
      // new PhotonPoseEstimator(
      // aprilTagFieldLayout,
      // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      // photonCam_2,
      // robotToCam_2);
    }

    // printing purposes
    df.setMaximumFractionDigits(2);
  }

  @Override
  public void periodic() {
    /* Ensures empty json not fed to pipeline */
    apriltagLimelightConnected = !NetworkTableInstance.getDefault()
        .getTable(VisionConfig.POSE_LIMELIGHT)
        .getEntry("json")
        .getString("")
        .equals("");

    NNLimelightConnected = !NetworkTableInstance.getDefault()
        .getTable(VisionConfig.NN_LIMELIGHT)
        .getEntry("json")
        .getString("")
        .equals("");

    if (VisionConfig.IS_LIMELIGHT_MODE && apriltagLimelightConnected) {
      jsonResults = LimelightHelpers.getLatestResults(VisionConfig.POSE_LIMELIGHT);
      tempPose = LimelightHelpers.getBotPose2d_wpiBlue(VisionConfig.POSE_LIMELIGHT);
      if (visionAccurate(tempPose)) {
        // json dump more accurate?
        // Update Vision robotpose - need to read more about coordinate systems centered
        // Blue alliance means origin is bottom right of the field
        limeLatency = LimelightHelpers.getLatency_Pipeline(VisionConfig.POSE_LIMELIGHT)
            + LimelightHelpers.getLatency_Capture(VisionConfig.POSE_LIMELIGHT);
        botPose = tempPose;
        // Shuffleboard Telemetry
        visionXDataEntry.setDouble(botPose.getX());
        visionYDataEntry.setDouble(botPose.getY());
        visionRotDataEntry.setDouble(botPose.getRotation().getDegrees());

      }

      // aimHorizontalOffset = jsonResults.results.getTX();
      // aimVerticalOffset = jsonResults.getTY();
      // aimTarget = LimelightHelpers.getTV(VisionConfig.POSE_LIMELIGHT);

      // Robot.log.logger.recordOutput("aimLL-VertOffset", aimVerticalOffset);
      // RobotTelemetry.print("aimLL-VertOffset: " + aimVerticalOffset);
    }

    // this method can call update() if vision pose estimation needs to be updated
    // in
    // Vision.java

    // Photonvision Result
    // The documentation for this is here:
    // https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html
    // The example code was missing, and we came up with this:
    // NOTE - PHOTONVISON GIVES POSES WITH BLUE ALLIANCE AS THE ORIGN ALWAYS!!!
    if (VisionConfig.IS_PHOTON_VISION_MODE) {

      var result_1 = photonCam_1.getLatestResult();
      photon1HasTargets = result_1.hasTargets();
      // var result_2 = photonCam_2.getLatestResult();
      // photon2HasTargets = result_2.hasTargets();

      if (photon1HasTargets) {
        var update = photonPoseEstimator.update();
        Pose3d currentPose3d = update.get().estimatedPose;
        botPose = currentPose3d.toPose2d();
        photonTimestamp = update.get().timestampSeconds;
      }
      // else if (photon2HasTargets){
      // var update = photonPoseEstimator_2.update();
      // Pose3d currentPose3d = update.get().estimatedPose;
      // botPose = currentPose3d.toPose2d();
      // photonTimestamp = update.get().timestampSeconds;
      // }

      visionXDataEntry.setDouble(botPose.getX());
      visionYDataEntry.setDouble(botPose.getY());
      visionRotDataEntry.setDouble(botPose.getRotation().getDegrees());
    }
  }

  /**
   * @return Pose2d location of note Field Relative
   */
  public Pose2d getNotePose2d() {
    return noteFieldRelativePose;
  }

  /**
   * @return Timestamp of photonvision's latest reading
   */
  public double getPhotonTimestamp() {
    return photonTimestamp;
  }

  /**
   * @return boolean if photonvision has targets
   */
  public boolean photonHasTargets() {
    if (photon1HasTargets) {
      return true;
    }
    return false;
  }

  /**
   * @return RobotPose2d with the apriltag as the origin (for chase apriltag
   *         command)
   */
  public Pose2d getRobotPose2d_TargetSpace() {
    return LimelightHelpers.getBotPose2d_TargetSpace(VisionConfig.POSE_LIMELIGHT);
  }

  /**
   * @return Pose2d of the apriltag with the robot as the origin
   */
  public Pose2d getTargetRobotPose_RobotSpace() {
    return LimelightHelpers.getTargetPose2d_RobotSpace(VisionConfig.POSE_LIMELIGHT);
  }

  // APRILTAG HELPER METHODS

  /**
   * @return if vision should be trusted more than estimated pose
   */
  public boolean visionAccurate(Pose2d currentPose) {
    return isValidPose() && (isInMap(currentPose) || multipleTargetsInView());
  }

  /**
   * @return whether or not vision sees a tag
   */
  public boolean isValidPose() {
    /* Disregard Vision if there are no targets in view */
    if (VisionConfig.IS_LIMELIGHT_MODE) {
      return LimelightHelpers.getTV(VisionConfig.POSE_LIMELIGHT);
    }
    if (VisionConfig.IS_PHOTON_VISION_MODE) {
      return photonHasTargets();
    }
    return false;
  }

  // This is a suss function - need to test it
  public boolean isInMap(Pose2d currentPose) {
    return ((currentPose.getX() > 0.0 && currentPose.getX() <= VisionConfig.FIELD_LENGTH_METERS)
        && (currentPose.getY() > 0.0 && currentPose.getY() <= VisionConfig.FIELD_WIDTH_METERS));
  }

  /**
   * @return whether the camera sees multiple tags or not
   */
  public boolean multipleTargetsInView() {
    if (jsonResults == null) {
      return false;
    }
    LimelightTarget_Fiducial[] tags = jsonResults.targetingResults.targets_Fiducials;
    if (tags.length > 1) {
      return true;
    }
    return false;
  }

  // Getter for visionBotPose - NEED TO DO TESTING TO MAKE SURE NO NULL ERRORS

  public Pose2d visionBotPose() {
    return botPose;
  }

  /**
   * @return the total latency of the limelight camera
   */
  public double getTotalLatency() {
    return limeLatency;
  }

  /**
   * Gets the camera capture time in seconds. Only used for limelight
   *
   * @param latencyMillis the latency of the camera in milliseconds
   * @return the camera capture time in seconds
   */
  public double getTimestampSeconds(double latencyMillis) {
    return Timer.getFPGATimestamp() - (latencyMillis / 1000d);
  }

  /**
   * @param limelight     name of limelight to control in {@link VisionConfig}
   * @param pipelineIndex use pipeline indexes in {@link VisionConfig}
   */
  public void setLimelightPipeline(String limelight, int pipelineIndex) {
    LimelightHelpers.setPipelineIndex(limelight, pipelineIndex);
  }

  /**
   * Gets target distance from the camera
   * 
   * @param cameraHeight               distance from lens to floor of camera in
   *                                   meters
   * @param cameraAngle                pitch of camera in radians
   * @param targetHeight               distance from floor to center of target in
   *                                   meters
   * @param targetOffsetAngle_Vertical ty entry from limelight of target crosshair
   *                                   (in degrees)
   * @return the distance to the target in meters
   */
  public double targetDistanceMetersCamera(
      double cameraHeight,
      double cameraAngle,
      double targetHeight,
      double targetOffsetAngle_Vertical) {
    double angleToGoalRadians = cameraAngle + targetOffsetAngle_Vertical * (3.14159 / 180.0);
    return (targetHeight - cameraHeight) / Math.tan(angleToGoalRadians);
  }

  /**
   * @param targetDistanceMeters         component of distance from camera to
   *                                     target
   * @param targetOffsetAngle_Horizontal tx entry from limelight of target
   *                                     crosshair (in degrees)
   * @return the translation to the target in meters
   */
  public Translation2d estimateCameraToTargetTranslation(double targetDistanceMeters,
      double targetOffsetAngle_Horizontal) {
    Rotation2d yaw = Rotation2d.fromDegrees(targetOffsetAngle_Horizontal);
    return new Translation2d(
        yaw.getCos() * targetDistanceMeters, yaw.getSin() * targetDistanceMeters);
  }

  /**
   * @param cameraToTargetTranslation2d  the translation from estimate camera to
   *                                     target
   * @param targetOffsetAngle_Horizontal tx entry from limelight of target
   *                                     crosshair (in degrees)
   * @return the position of the target in terms of the camera
   */
  public Pose2d estimateCameraToTargetPose2d(Translation2d cameraToTargetTranslation2d,
      double targetOffsetAngle_Horizontal) {
    return new Pose2d(cameraToTargetTranslation2d, Rotation2d.fromDegrees(targetOffsetAngle_Horizontal));
  }

  /**
   * @param camToTargetPose the camera to target pose 2d
   * @param camToRobot      the transform from the x and y of the camera to the
   *                        center of the robot
   * @return the position of the target relative to the robot
   */
  public Pose2d camPoseToRobotRelativeTargetPose2d(Pose2d camToTargetPose, Transform2d camToRobot) {
    return camToTargetPose.transformBy(camToRobot);

  }

  /**
   * RobotRelativePose of the current target
   * 
   * @return the position of the target relative to the robot
   */
  public Pose2d targetPoseRobotSpace() {
    return targetRobotRelativePose;
  }

  /**
   * @param notePoseRobotRelative the RobotRelative Pose2d of the note
   * @param botPoseFieldRelative  The FieldRelative Pose2d of the robot
   * @return the FieldRelative Pose2d of the note
   */
  public Pose2d notePoseFieldSpace(Pose2d notePoseRobotRelative, Pose2d botPoseFieldRelative) {
    Transform2d noteTransform = new Transform2d(notePoseRobotRelative.getTranslation(),
        notePoseRobotRelative.getRotation());
    Pose2d notePose = botPoseFieldRelative.transformBy(noteTransform);
    return notePose;
  }

  /**
   * Commnad to go to the note
   * 
   * @return a follow path command to drive to the note
   */
  public Command followNoteCommand() {
    // return Swerve.getInstance().onTheFlyPathCommand(this::getNotePose2d); //
    // doing this::getNotePose2d converts to a
    return null; // supplier
  }

  /**
   * Prints the vision, estimated, and odometry pose to SmartDashboard
   *
   * @param values the array of limelight raw values
   */
  /*
   * public void printDebug(double[] poseArray) {
   * if (poseArray.length > 0) {
   * SmartDashboard.putString("LimelightX",
   * df.format(botPose3d.getTranslation().getX()));
   * SmartDashboard.putString("LimelightY",
   * df.format(botPose3d.getTranslation().getY()));
   * SmartDashboard.putString("LimelightZ",
   * df.format(botPose3d.getTranslation().getZ()));
   * SmartDashboard.putString(
   * "LimelightRoll",
   * df.format(Units.radiansToDegrees(botPose3d.getRotation().getX())));
   * SmartDashboard.putString(
   * "LimelightPitch",
   * df.format(Units.radiansToDegrees(botPose3d.getRotation().getY())));
   * SmartDashboard.putString(
   * "LimelightYaw",
   * df.format(Units.radiansToDegrees(botPose3d.getRotation().getZ())));
   * }
   * SmartDashboard.putString("EstimatedPoseX",
   * df.format(Robot.pose.getEstimatedPose().getX()));
   * SmartDashboard.putString("EstimatedPoseY",
   * df.format(Robot.pose.getEstimatedPose().getY()));
   * SmartDashboard.putString(
   * "EstimatedPoseTheta", df.format(Robot.pose.getHeading().getDegrees()));
   * }
   */
}

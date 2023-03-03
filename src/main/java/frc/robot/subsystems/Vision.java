// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  //Swerve swerve = new Swerve();
  double tagID;
  double tv;
  Swerve swerve = new Swerve();
  private static SwerveDrivePoseEstimator poseEstimator;
  private static List<PathPlannerTrajectory> Auto3 = PathPlanner.loadPathGroup("Auto 3", new PathConstraints(0.25, 0.25));
  private final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  private final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.02, 0.02, Units.degreesToRadians(5));
  
  public Vision(Swerve swerve) {
    this.swerve = swerve;
    poseEstimator = new SwerveDrivePoseEstimator(
      Constants.Swerve.swerveKinematics, 
      swerve.getYaw(),//Rotation2d.fromDegrees(swerve.gyro.getYaw()), 
      swerve.getModulePositions(), 
      Auto3.get(0).getInitialPose(), 
      stateStdDevs, 
      visionMeasurementStdDevs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumberArray("Botpose", limeOdometry());
    // double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    // double dist = Constants.VisionConstants.aprilLow/(Math.tan(Math.toRadians(ty)));
    // SmartDashboard.putNumber("DistFrmTy", dist);

    // double[] limeInfo = getLime3Data();
    // double x = limeInfo[0];
    // double y = limeInfo[1];
    // double z = limeInfo[2];
    // double roll = limeInfo[3];
    // double pitch = limeInfo[4];
    // double yaw = limeInfo[5];
    // SmartDashboard.putNumber("LimeX", x);
    // SmartDashboard.putNumber("LimeY", y);
    // SmartDashboard.putNumber("LimeZ", z);
    // SmartDashboard.putNumber("LimeRoll", roll);
    // SmartDashboard.putNumber("LimePitch", pitch);
    // SmartDashboard.putNumber("LimeYaw", yaw);
    

    dashboard();

    

  }


  /*public Translation2d manualLimeOdometry()
  {
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double dist = Constants.VisionConstants.aprilLow/(Math.tan(Math.toRadians(ty)));
    double thor = NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor").getDouble(0);
    double theta = Math.toDegrees(Math.acos(thor/.15));
    //Translation2d robotToTarget = new Translation2d(tx)
  }*/

  public double[] getLime3Data()
  {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
  }

  public void dashboard()
  {
    double timestamp = Timer.getFPGATimestamp();
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    tagID = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);
    if(tv==1.0)
    {
      //swerve.resetOdometry(lime2Robot(getLime3Data(), (int)tagID));
      Pose2d tempPos = lime2Robot(getLime3Data(), (int)tagID);
      poseEstimator.addVisionMeasurement(tempPos, timestamp);
      SmartDashboard.putString("LIME POSE", tempPos.toString());
    }

    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), swerve.getYaw(), swerve.getModulePositions());
    SmartDashboard.putString("PE POSE", poseEstimator.getEstimatedPosition().toString());
    swerve.resetOdometry(poseEstimator.getEstimatedPosition());
  }

  public Pose2d lime2Robot(double[] limeInfo, int tagID)
  {
    double x = limeInfo[0];
    //double y = limeInfo[1];
    double z = limeInfo[2];
    //double roll = limeInfo[3];
    double pitch = limeInfo[4];
    //double yaw = limeInfo[5];

    double tar2robotX = x+Constants.VisionConstants.LimeOffCenterX;
    double tar2robotZ = z+Constants.VisionConstants.LimeOffCenterZ;

    double tagX = Constants.VisionConstants.tag[tagID-1][0];
    double tagY = Constants.VisionConstants.tag[tagID-1][1];

    double robotRot = Constants.VisionConstants.tag[tagID-1][2] - pitch;

    double poseX, poseY;

    if (robotRot > 90)
    {
      poseX = tagX+tar2robotZ;
      poseY = tagY-tar2robotX;
    }
    else
    {
      poseX = tagX-tar2robotZ;
      poseY = tagY+tar2robotX;
    }

    Pose2d robotPose = new Pose2d(new Translation2d(poseX, poseY), new Rotation2d(Math.toRadians(robotRot)));
    return robotPose;
  }




}

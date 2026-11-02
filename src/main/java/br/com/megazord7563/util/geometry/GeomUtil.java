package br.com.megazord7563.util.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Geometry utilities for working with translations, rotations, transforms, and poses in 2D and
 * 3D.
 */
public class GeomUtil {

  private GeomUtil() {}

  // ---------------------------------------------------------------------
  // Transform2d
  // ---------------------------------------------------------------------

  /** Creates a purely translational 2D transform. */
  public static Transform2d toTransform2d(Translation2d translation) {
    return new Transform2d(translation, Rotation2d.kZero);
  }

  /** Creates a purely translational 2D transform from x and y coordinates. */
  public static Transform2d toTransform2d(double x, double y) {
    return new Transform2d(x, y, Rotation2d.kZero);
  }

  /** Creates a purely rotational 2D transform. */
  public static Transform2d toTransform2d(Rotation2d rotation) {
    return new Transform2d(Translation2d.kZero, rotation);
  }

  /** Converts a Pose2d into a Transform2d, useful in kinematic chains. */
  public static Transform2d toTransform2d(Pose2d pose) {
    return new Transform2d(pose.getTranslation(), pose.getRotation());
  }

  /** Projects a Transform3d onto a Transform2d, discarding Z and out-of-plane rotations. */
  public static Transform2d toTransform2d(Transform3d transform) {
    return new Transform2d(
        transform.getTranslation().toTranslation2d(), transform.getRotation().toRotation2d());
  }

  // ---------------------------------------------------------------------
  // Pose2d
  // ---------------------------------------------------------------------

  /** Converts a Transform2d into a Pose2d, useful as the origin of a kinematic chain. */
  public static Pose2d toPose2d(Transform2d transform) {
    return new Pose2d(transform.getTranslation(), transform.getRotation());
  }

  /** Creates a purely translational 2D pose. */
  public static Pose2d toPose2d(Translation2d translation) {
    return new Pose2d(translation, Rotation2d.kZero);
  }

  /** Creates a purely rotational 2D pose. */
  public static Pose2d toPose2d(Rotation2d rotation) {
    return new Pose2d(Translation2d.kZero, rotation);
  }

  /** Projects a Pose3d onto a Pose2d. */
  public static Pose2d toPose2d(Pose3d pose) {
    return new Pose2d(pose.getTranslation().toTranslation2d(), pose.getRotation().toRotation2d());
  }

  /** Returns the inverse of a Pose2d (equivalent to moving "backward" through the transform). */
  public static Pose2d inverse(Pose2d pose) {
    Rotation2d rotationInverse = pose.getRotation().unaryMinus();
    return new Pose2d(
        pose.getTranslation().unaryMinus().rotateBy(rotationInverse), rotationInverse);
  }

  /** Creates a new pose from an existing one, changing only the translation. */
  public static Pose2d withTranslation(Pose2d pose, Translation2d translation) {
    return new Pose2d(translation, pose.getRotation());
  }

  /** Creates a new pose from an existing one, changing only the rotation. */
  public static Pose2d withRotation(Pose2d pose, Rotation2d rotation) {
    return new Pose2d(pose.getTranslation(), rotation);
  }

  // ---------------------------------------------------------------------
  // Transform3d
  // ---------------------------------------------------------------------

  /** Creates a purely translational 3D transform. */
  public static Transform3d toTransform3d(Translation3d translation) {
    return new Transform3d(translation, Rotation3d.kZero);
  }

  /** Creates a purely rotational 3D transform. */
  public static Transform3d toTransform3d(Rotation3d rotation) {
    return new Transform3d(Translation3d.kZero, rotation);
  }

  /** Converts a Pose3d into a Transform3d, useful in kinematic chains. */
  public static Transform3d toTransform3d(Pose3d pose) {
    return new Transform3d(pose.getTranslation(), pose.getRotation());
  }

  /** Lifts a Transform2d into a Transform3d (Z = 0, with no out-of-plane rotation). */
  public static Transform3d toTransform3d(Transform2d transform) {
    return new Transform3d(
        new Translation3d(transform.getX(), transform.getY(), 0.0),
        new Rotation3d(transform.getRotation()));
  }

  // ---------------------------------------------------------------------
  // Pose3d
  // ---------------------------------------------------------------------

  /** Converts a Transform3d into a Pose3d, useful as the origin of a kinematic chain. */
  public static Pose3d toPose3d(Transform3d transform) {
    return new Pose3d(transform.getTranslation(), transform.getRotation());
  }

  /** Lifts a Pose2d into a Pose3d (Z = 0, with no out-of-plane rotation). */
  public static Pose3d toPose3d(Pose2d pose) {
    return new Pose3d(
        new Translation3d(pose.getX(), pose.getY(), 0.0), new Rotation3d(pose.getRotation()));
  }

  /** Creates a purely translational 3D pose. */
  public static Pose3d toPose3d(Translation3d translation) {
    return new Pose3d(translation, Rotation3d.kZero);
  }

  /** Creates a purely rotational 3D pose. */
  public static Pose3d toPose3d(Rotation3d rotation) {
    return new Pose3d(Translation3d.kZero, rotation);
  }

  // ---------------------------------------------------------------------
  // Twist2d / Twist3d
  // ---------------------------------------------------------------------

  /** Multiplies a 2D twist by a scale factor. */
  public static Twist2d multiply(Twist2d twist, double factor) {
    return new Twist2d(twist.dx * factor, twist.dy * factor, twist.dtheta * factor);
  }

  /** Multiplies a 3D twist by a scale factor. */
  public static Twist3d multiply(Twist3d twist, double factor) {
    return new Twist3d(
        twist.dx * factor,
        twist.dy * factor,
        twist.dz * factor,
        twist.rx * factor,
        twist.ry * factor,
        twist.rz * factor);
  }

  /** Converts ChassisSpeeds into a Twist2d (useful for odometry integration). */
  public static Twist2d toTwist2d(ChassisSpeeds speeds) {
    return new Twist2d(
        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  // ---------------------------------------------------------------------
  // Velocities
  // ---------------------------------------------------------------------

  /**
   * Transforms a velocity (ChassisSpeeds) from one point on the robot to another, given the offset
   * between them and the robot's current rotation. Useful for calculating the velocity of a mechanism
   * offset from the robot's center (e.g., a swerve module or a camera).
   *
   * @param velocity The original velocity (at the reference point)
   * @param transform The offset to the new position
   * @param currentRotation The robot's current rotation
   * @return The equivalent velocity at the new point
   */
  public static ChassisSpeeds transformVelocity(
      ChassisSpeeds velocity, Translation2d transform, Rotation2d currentRotation) {
    return new ChassisSpeeds(
        velocity.vxMetersPerSecond
            - velocity.omegaRadiansPerSecond
                * (transform.getX() * currentRotation.getSin()
                    + transform.getY() * currentRotation.getCos()),
        velocity.vyMetersPerSecond
            + velocity.omegaRadiansPerSecond
                * (transform.getX() * currentRotation.getCos()
                    - transform.getY() * currentRotation.getSin()),
        velocity.omegaRadiansPerSecond);
  }
}
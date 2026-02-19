package org.firstinspires.ftc.teamcode.subsystem

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.skeletonarmy.marrow.zones.Point
import com.skeletonarmy.marrow.zones.PolygonZone
import kotlin.math.cos
import kotlin.math.sin

/**
 * Drivetrain wrapper around PedroPathing Follower
 * Provides easy access to pose, velocity, and zone checking
 */
class Drivetrain(private val follower: Follower) {
    
    /**
     * Get current pose (x, y, heading in radians)
     */
    fun pose(): Pose = follower.pose
    
    /**
     * Get X position in inches
     */
    fun x(): Double = follower.pose.x
    
    /**
     * Get Y position in inches
     */
    fun y(): Double = follower.pose.y
    
    /**
     * Get heading in radians
     */
    fun heading(): Double = follower.pose.heading
    
    /**
     * Get heading in degrees
     */
    fun headingDegrees(): Double = Math.toDegrees(follower.pose.heading)
    
    /**
     * Get total heading (for continuous rotation)
     */
    fun totalHeading(): Double = follower.totalHeading
    
    /**
     * Get velocity X component (inches/sec)
     */
    fun velocityX(): Double = follower.velocity.xComponent
    
    /**
     * Get velocity Y component (inches/sec)
     */
    fun velocityY(): Double = follower.velocity.yComponent
    
    /**
     * Get angular velocity (radians/sec)
     */
    fun angularVelocity(): Double = follower.angularVelocity
    
    /**
     * Get speed magnitude (inches/sec)
     */
    fun speed(): Double = follower.velocity.magnitude()
    
    /**
     * Update follower (call this in periodic)
     */
    fun update() {
        follower.update()
    }
    
    /**
     * Set starting pose for localization
     */
    fun setStartingPose(x: Double, y: Double, heading: Double) {
        follower.setStartingPose(Pose(x, y, heading))
    }
    
    /**
     * Set starting pose from Pose object
     */
    fun setStartingPose(pose: Pose) {
        follower.setStartingPose(pose)
    }
    
    /**
     * Reset pose to specific position
     */
    fun resetPose(x: Double, y: Double, heading: Double) {
        follower.setPose(x, y, heading)
    }
    
    /**
     * Reset to origin
     */
    fun reset() {
        follower.setPose(0.0, 0.0, 0.0)
    }
    
    /**
     * Start following a path
     */
    fun followPath(path: com.pedropathing.paths.Path, interruptable: Boolean = true) {
        follower.followPath(path, interruptable)
    }
    
    /**
     * Check if currently following a path
     */
    fun isBusy(): Boolean = follower.isBusy()
    
    /**
     * Stop following path
     */
    fun stopFollowing() {
        follower.stopFollowing()
    }
    
    /**
     * Turn to specific angle (radians)
     */
    fun turnTo(angle: Double) {
        follower.turnToAngle(angle)
    }
    
    /**
     * Turn to angle in degrees
     */
    fun turnToDegrees(angleDegrees: Double) {
        follower.turnToAngle(Math.toRadians(angleDegrees))
    }
    
    /**
     * Set motor powers directly
     */
    fun setMotorPowers(frontLeft: Double, frontRight: Double, backLeft: Double, backRight: Double) {
        follower.setMotorPowers(frontLeft, frontRight, backLeft, backRight)
    }
    
    /**
     * Stop all motors
     */
    fun stop() {
        follower.stop()
    }
}

/**
 * Shooting zone using Skeleton Marrow PolygonZone
 */
class ShootingZone(private val points: List<Point>) {
    
    private val polygonZone = PolygonZone(*points.toTypedArray())
    
    /**
     * Check if pose is inside zone
     */
    fun contains(pose: Pose): Boolean = polygonZone.contains(pose)
    
    /**
     * Check if x,y is inside zone
     */
    fun contains(x: Double, y: Double): Boolean = polygonZone.contains(x, y)
    
    /**
     * Get center of zone
     */
    fun center(): Point = polygonZone.center
}

/**
 * Predefined shooting zones
 */
object ShootingZones {
    
    /**
     * Close launch zone (near goal) - adjust coordinates to match your field
     */
    val closeZone = ShootingZone(listOf(
        Point(144.0, 144.0),  // Top-right (goal area)
        Point(72.0, 72.0),     // Field center  
        Point(0.0, 144.0)      // Top-left
    ))
    
    /**
     * Far launch zone (auto line)
     */
    val farZone = ShootingZone(listOf(
        Point(48.0, 0.0),     // Bottom-left
        Point(72.0, 24.0),    // Middle
        Point(96.0, 0.0)      // Bottom-right
    ))
    
    /**
     * Check if robot is in close zone
     */
    fun inCloseZone(drivetrain: Drivetrain): Boolean {
        return closeZone.contains(drivetrain.pose())
    }
    
    /**
     * Check if robot is in far zone
     */
    fun inFarZone(drivetrain: Drivetrain): Boolean {
        return farZone.contains(drivetrain.pose())
    }
    
    /**
     * Get which zone robot is in
     */
    fun getZone(drivetrain: Drivetrain): String {
        return when {
            inCloseZone(drivetrain) -> "CLOSE"
            inFarZone(drivetrain) -> "FAR"
            else -> "NONE"
        }
    }
}

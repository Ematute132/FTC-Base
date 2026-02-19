package org.firstinspires.ftc.teamcode.subsystem

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.skeletonarmy.marrow.zones.Point
import com.skeletonarmy.marrow.zones.PolygonZone
import kotlin.math.cos
import kotlin.math.sin

/**
 * Complete Drive subsystem using PedroPathing Follower
 * Includes pose tracking, velocity, and zone detection
 */
class Drive(private val follower: Follower) {
    
    // ==================== POSE ====================
    
    /**
     * Current robot pose - always returns latest position
     * Use this instead of pose() for real-time tracking
     */
    val currentPose: Pose
        get() = follower.pose
    
    /**
     * Get X position in inches
     */
    val x: Double
        get() = follower.pose.x
    
    /**
     * Get Y position in inches
     */
    val y: Double
        get() = follower.pose.y
    
    /**
     * Get heading in radians
     */
    val heading: Double
        get() = follower.pose.heading
    
    /**
     * Get heading in degrees
     */
    val headingDegrees: Double
        get() = Math.toDegrees(follower.pose.heading)
    
    /**
     * Get total heading (continuous rotation)
     */
    val totalHeading: Double
        get() = follower.totalHeading
    
    // ==================== VELOCITY ====================
    
    /**
     * Velocity X component (inches/sec)
     */
    val velocityX: Double
        get() = follower.velocity.xComponent
    
    /**
     * Velocity Y component (inches/sec)
     */
    val velocityY: Double
        get() = follower.velocity.yComponent
    
    /**
     * Angular velocity (radians/sec)
     */
    val angularVelocity: Double
        get() = follower.angularVelocity
    
    /**
     * Speed magnitude (inches/sec)
     */
    val speed: Double
        get() = follower.velocity.magnitude()
    
    // ==================== UPDATE ====================
    
    /**
     * Update follower - call this in periodic/update loop
     */
    fun update() {
        follower.update()
    }
    
    // ==================== POSE CONTROL ====================
    
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
     * Reset pose to origin
     */
    fun reset() {
        follower.setPose(0.0, 0.0, 0.0)
    }
    
    /**
     * Reset pose to specific position
     */
    fun resetPose(x: Double, y: Double, heading: Double) {
        follower.setPose(x, y, heading)
    }
    
    // ==================== PATH FOLLOWING ====================
    
    /**
     * Start following a path
     * @param path Path to follow
     * @param interruptable Whether gamepad can interrupt
     */
    fun followPath(path: com.pedropathing.paths.Path, interruptable: Boolean = true) {
        follower.followPath(path, interruptable)
    }
    
    /**
     * Check if currently following a path
     */
    val isBusy: Boolean
        get() = follower.isBusy()
    
    /**
     * Stop following current path
     */
    fun stopFollowing() {
        follower.stopFollowing()
    }
    
    /**
     * Turn to angle in radians
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
    
    // ==================== MOTOR CONTROL ====================
    
    /**
     * Set motor powers directly
     */
    fun setMotorPowers(frontLeft: Double, frontRight: Double, backLeft: Double, backRight: Double) {
        follower.setMotorPowers(frontLeft, frontRight, backLeft, backRight)
    }
    
    /**
     * Stop all drive motors
     */
    fun stop() {
        follower.stop()
    }
}

/**
 * Zone detection using Skeleton Marrow PolygonZone
 */
class Zone(private val points: List<Point>) {
    
    private val polygon = PolygonZone(*points.toTypedArray())
    
    /**
     * Check if pose is inside zone
     */
    fun contains(pose: Pose): Boolean = polygon.contains(pose)
    
    /**
     * Check if x,y is inside zone
     */
    fun contains(x: Double, y: Double): Boolean = polygon.contains(x, y)
    
    /**
     * Get center point of zone
     */
    val center: Point
        get() = polygon.center
}

/**
 * Predefined shooting zones
 * Adjust coordinates to match your field/hub position
 */
object Zones {
    
    // Field dimensions (FTC standard: 141.5" x 141.5")
    const val FIELD_WIDTH = 141.5
    const val FIELD_HEIGHT = 141.5
    
    // ==================== SHOOTING ZONES ====================
    
    /**
     * Close shooting zone (near hub)
     */
    val closeZone = Zone(listOf(
        Point(144.0, 144.0),  // Top-right (goal area)
        Point(72.0, 72.0),    // Field center
        Point(0.0, 144.0)     // Top-left
    ))
    
    /**
     * Far shooting zone (auto line)
     */
    val farZone = Zone(listOf(
        Point(48.0, 0.0),    // Bottom-left
        Point(72.0, 24.0),   // Middle
        Point(96.0, 0.0)     // Bottom-right
    ))
    
    /**
     * Custom zone - configure your own
     */
    fun customZone(points: List<Point>): Zone = Zone(points)
    
    // ==================== ZONE CHECKS ====================
    
    /**
     * Check if robot is in close shooting zone
     */
    fun inCloseZone(drive: Drive): Boolean = closeZone.contains(drive.currentPose)
    
    /**
     * Check if robot is in far shooting zone
     */
    fun inFarZone(drive: Drive): Boolean = farZone.contains(drive.currentPose)
    
    /**
     * Get which zone robot is in
     * @return "CLOSE", "FAR", or "NONE"
     */
    fun getZone(drive: Drive): String = when {
        inCloseZone(drive) -> "CLOSE"
        inFarZone(drive) -> "FAR"
        else -> "NONE"
    }
    
    /**
     * Distance from robot to close zone center
     */
    fun distanceToCloseZone(drive: Drive): Double {
        val pose = drive.currentPose
        val center = closeZone.center
        return kotlin.math.sqrt(
            (pose.x - center.x).pow(2) + 
            (pose.y - center.y).pow(2)
        )
    }
    
    /**
     * Distance from robot to far zone center
     */
    fun distanceToFarZone(drive: Drive): Double {
        val pose = drive.currentPose
        val center = farZone.center
        return kotlin.math.sqrt(
            (pose.x - center.x).pow(2) + 
            (pose.y - center.y).pow(2)
        )
    }
}

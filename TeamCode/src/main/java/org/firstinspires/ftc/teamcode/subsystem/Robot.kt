package org.firstinspires.ftc.teamcode.subsystem

import com.pedropathing.geometry.Pose
import com.pedropathing.follower.Follower
import com.skeletonarmy.marrow.zones.Point
import com.skeletonarmy.marrow.zones.PolygonZone
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import kotlin.math.cos
import kotlin.math.sin

/**
 * Robot state object using Pedro + Skeleton Marrow zones
 */
object Robot {
    // Field dimensions
    const val FIELD_WIDTH: Double = 141.5
    const val FIELD_LENGTH: Double = 141.5
    
    // Turret offset from robot center
    const val TURRET_Y_OFFSET: Double = -0.6
    
    // Zone buffers
    const val FAR_ZONE_BUFFER: Double = 12.0
    const val CLOSE_ZONE_BUFFER: Double = 11.0
    
    // Robot dimensions
    const val ROBOT_WIDTH = 18.0
    const val ROBOT_LENGTH = 18.0
    
    /**
     * Get current robot pose from Pedro
     */
    fun pose(): Pose = follower.pose
    
    /**
     * Get shooter pose (accounting for turret offset)
     */
    fun shooterPose(): Pose {
        val a = follower.heading
        return follower.pose + Pose(
            TURRET_Y_OFFSET * cos(a),
            TURRET_Y_OFFSET * sin(a)
        )
    }
    
    /**
     * Predict pose at future time (for shooting adjustment)
     */
    fun correctedPose(predictionTime: Double): Pose {
        val v = follower.velocity
        val predictedPose = Pose(
            v.xComponent * predictionTime,
            v.yComponent * predictionTime,
            follower.angularVelocity * predictionTime
        )
        return shooterPose() + predictedPose
    }
    
    /**
     * Get velocity components
     */
    fun velocityX(): Double = follower.velocity.xComponent
    fun velocityY(): Double = follower.velocity.yComponent
    fun angularVelocity(): Double = follower.angularVelocity
    
    /**
     * Get speed magnitude
     */
    fun speed(): Double = follower.velocity.magnitude()
    
    /**
     * Get heading in radians
     */
    fun heading(): Double = follower.heading
    
    /**
     * Reset pose to specific position
     */
    fun resetPose(x: Double, y: Double, heading: Double) {
        follower.resetPose(Pose(x, y, heading))
    }
}

/**
 * Shooting zones using Skeleton Marrow PolygonZone
 */
object ShootingZones {
    
    /**
     * Close launch zone (near goal)
     */
    val closeLaunchZone = PolygonZone(
        Point(144.0, 144.0),  // Top-right (goal area)
        Point(72.0, 72.0),     // Field center
        Point(0.0, 144.0)      // Top-left
    )
    
    /**
     * Far launch zone (auto line)
     */
    val farLaunchZone = PolygonZone(
        Point(48.0, 0.0),     // Bottom-left
        Point(72.0, 24.0),    // Middle
        Point(96.0, 0.0)       // Bottom-right
    )
    
    /**
     * Robot collision zone
     */
    val robotZone = PolygonZone(18.0, 18.0) // width, length
    
    /**
     * Check if robot is in close launch zone
     */
    fun inCloseZone(): Boolean = closeLaunchZone.contains(Robot.pose())
    
    /**
     * Check if robot is in far launch zone
     */
    fun inFarZone(): Boolean = farLaunchZone.contains(Robot.pose())
    
    /**
     * Get which zone robot is in
     */
    fun getZone(): String {
        return when {
            closeLaunchZone.contains(Robot.pose()) -> "CLOSE"
            farLaunchZone.contains(Robot.pose()) -> "FAR"
            else -> "NONE"
        }
    }
}

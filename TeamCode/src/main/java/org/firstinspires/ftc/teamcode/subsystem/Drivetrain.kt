package org.firstinspires.ftc.teamcode.subsystem

import com.pedropathing.follower.Follower
import com.pedropathing.localization.Pose
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.pow

/**
 * 2D Pose wrapper around Pedro's Pose
 */
data class RobotPose(
    var x: Double = 0.0,
    var y: Double = 0.0,
    var heading: Double = 0.0
) {
    fun toPedro(): Pose = Pose(x, y, heading)
    
    companion object {
        fun fromPedro(pose: Pose): RobotPose = RobotPose(pose.getX(), pose.getY(), pose.getHeading())
    }
    
    fun distanceTo(other: RobotPose): Double {
        return sqrt((x - other.x).pow(2) + (y - other.y).pow(2))
    }
}

/**
 * Shooting zone defined by polygon vertices
 * Uses ray casting algorithm for point-in-polygon detection
 */
class ShootingZone(private val vertices: List<RobotPose>) {
    
    /**
     * Check if a point is inside the polygon using ray casting
     */
    fun contains(pose: RobotPose): Boolean = contains(pose.x, pose.y)
    
    fun contains(x: Double, y: Double): Boolean {
        if (vertices.size < 3) return false
        
        var inside = false
        var j = vertices.size - 1
        
        for (i in vertices.indices) {
            val xi = vertices[i].x
            val yi = vertices[i].y
            val xj = vertices[j].x
            val yj = vertices[j].y
            
            val intersect = ((yi > y) != (yj > y)) &&
                    (x < (xj - xi) * (y - yi) / (yj - yi) + xi)
            
            if (intersect) inside = !inside
            j = i
        }
        
        return inside
    }
    
    fun center(): RobotPose {
        val avgX = vertices.map { it.x }.average()
        val avgY = vertices.map { it.y }.average()
        return RobotPose(avgX, avgY)
    }
    
    fun distanceTo(pose: RobotPose): Double {
        if (contains(pose)) return 0.0
        
        var minDist = Double.MAX_VALUE
        for (i in vertices.indices) {
            val j = (i + 1) % vertices.size
            val dist = pointToLineSegmentDistance(
                pose.x, pose.y,
                vertices[i].x, vertices[i].y,
                vertices[j].x, vertices[j].y
            )
            minDist = min(minDist, dist)
        }
        return minDist
    }
    
    private fun pointToLineSegmentDistance(
        px: Double, py: Double,
        x1: Double, y1: Double,
        x2: Double, y2: Double
    ): Double {
        val dx = x2 - x1
        val dy = y2 - y1
        val lengthSq = dx * dx + dy * dy
        
        if (lengthSq == 0.0) return sqrt((px - x1).pow(2) + (py - y1).pow(2))
        
        var t = ((px - x1) * dx + (py - y1) * dy) / lengthSq
        t = t.coerceIn(0.0, 1.0)
        
        val nearX = x1 + t * dx
        val nearY = y1 + t * dy
        
        return sqrt((px - nearX).pow(2) + (py - nearY).pow(2))
    }
}

/**
 * Drivetrain using PedroPathing for odometry
 * Wraps Pedro's follower for pose tracking
 */
class Drivetrain(private val follower: Follower) {
    
    /**
     * Get current robot pose from Pedro
     */
    fun getPose(): RobotPose {
        val pose = follower.pose
        return RobotPose(pose.getX(), pose.getY(), pose.getHeading())
    }
    
    /**
     * Get velocity X (inches/sec)
     */
    fun getVelocityX(): Double = follower.velocityX
    
    /**
     * Get velocity Y (inches/sec)
     */
    fun getVelocityY(): Double = follower.velocityY
    
    /**
     * Get speed magnitude (inches/sec)
     */
    fun getSpeed(): Double = sqrt(follower.velocityX.pow(2) + follower.velocityY.pow(2))
    
    /**
     * Get angular velocity (radians/sec)
     */
    fun getAngularVelocity(): Double = follower.angularVelocity
    
    /**
     * Get heading in degrees
     */
    fun getHeadingDegrees(): Double = Math.toDegrees(follower.pose.getHeading())
    
    /**
     * Update Pedro follower (call this in periodic)
     */
    fun update() {
        follower.update()
    }
    
    /**
     * Reset pose to specific position
     */
    fun resetPose(x: Double, y: Double, heading: Double) {
        follower.resetPose(Pose(x, y, heading))
    }
    
    /**
     * Reset to origin
     */
    fun reset() {
        follower.resetPose(Pose(0.0, 0.0, 0.0))
    }
    
    /**
     * Start path following
     */
    fun followPath(path: com.pedropathing.paths.Path, interruptable: Boolean = true) {
        follower.followPath(path, interruptable)
    }
    
    /**
     * Check if currently following a path
     */
    fun isBusy(): Boolean = follower.isBusy()
    
    /**
     * Turn to specific angle
     */
    fun turnTo(angle: Double) {
        follower.turn(angle)
    }
    
    /**
     * Set drive motor powers directly
     */
    fun setDrivePowers(frontLeft: Double, frontRight: Double, backLeft: Double, backRight: Double) {
        follower.setMotorPowers(frontLeft, frontRight, backLeft, backRight)
    }
    
    /**
     * Stop all motors
     */
    fun stop() {
        follower.stop()
    }
}

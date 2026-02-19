package org.firstinspires.ftc.teamcode.subsystem

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.IMU
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.angle
import kotlin.math.*

/**
 * 2D Pose: position and heading
 */
data class Pose(
    var x: Double = 0.0,      // inches
    var y: Double = 0.0,      // inches
    var heading: Double = 0.0 // radians
) {
    fun distanceTo(other: Pose): Double {
        return sqrt((x - other.x).pow(2) + (y - other.y).pow(2))
    }
}

/**
 * Shooting zone defined by polygon vertices
 * Uses ray casting algorithm for point-in-polygon detection
 */
class ShootingZone(private val vertices: List<Pose>) {
    
    /**
     * Check if a point is inside the polygon using ray casting
     * Works for any convex or concave polygon
     */
    fun contains(pose: Pose): Boolean = contains(pose.x, pose.y)
    
    fun contains(x: Double, y: Double): Boolean {
        if (vertices.size < 3) return false
        
        var inside = false
        var j = vertices.size - 1
        
        for (i in vertices.indices) {
            val xi = vertices[i].x
            val yi = vertices[i].y
            val xj = vertices[j].x
            val yj = vertices[j].y
            
            // Ray casting algorithm
            val intersect = ((yi > y) != (yj > y)) &&
                    (x < (xj - xi) * (y - yi) / (yj - yi) + xi)
            
            if (intersect) inside = !inside
            j = i
        }
        
        return inside
    }
    
    /**
     * Get center of shooting zone (for distance calculations)
     */
    fun center(): Pose {
        val avgX = vertices.map { it.x }.average()
        val avgY = vertices.map { it.y }.average()
        return Pose(avgX, avgY)
    }
    
    /**
     * Distance from pose to nearest point in zone
     */
    fun distanceTo(pose: Pose): Double {
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
 * Drivetrain with odometry and pose tracking
 * Uses 3 dead wheels + IMU for accurate position/heading
 */
class Drivetrain : Subsystem {
    
    // Motors (configure your encoder ports)
    private val leftEncoder = MotorEx("encoderLeft")
    private val rightEncoder = MotorEx("encoderRight")
    private val strafeEncoder = MotorEx("encoderStrafe")
    
    // IMU for heading
    private val imu = IMU("imu")
    
    // Odometry wheel positions (in inches from robot center)
    private val leftOffset = -6.0   // adjust for your bot
    private val rightOffset = 6.0
    private val strafeOffset = 6.0
    
    // Wheel ticks per inch (calculate from your gear ratio)
    private val ticksPerInch = 317.97 // example: 537.7 ticks/rev / (4π inch/rev)
    
    // Current pose
    var pose = Pose()
        private set
    
    // Previous encoder values
    private var prevLeft = 0.0
    private var prevRight = 0.0
    private var prevStrafe = 0.0
    private var prevHeading = 0.0
    
    // Velocity tracking
    var velocityX = 0.0
        private set
    var velocityY = 0.0
        private set
    var angularVelocity = 0.0
        private set
    
    // Filter for smoothing
    private val velocityAlpha = 0.3
    
    override fun periodic() {
        // Get current encoder values
        val currentLeft = leftEncoder.currentPosition.toDouble()
        val currentRight = rightEncoder.currentPosition.toDouble()
        val currentStrafe = strafeEncoder.currentPosition.toDouble()
        
        // Get heading from IMU
        val currentHeading = imu.robotHeading.angle.toRadians()
        
        // Calculate deltas
        val dLeft = (currentLeft - prevLeft) / ticksPerInch
        val dRight = (currentRight - prevRight) / ticksPerInch
        val dStrafe = (currentStrafe - prevStrafe) / ticksPerInch
        val dHeading = currentHeading - prevHeading
        
        // Handle heading wraparound
        val dHeadingNormalized = when {
            dHeading > PI -> dHeading - 2 * PI
            dHeading < -PI -> dHeading + 2 * PI
            else -> dHeading
        }
        
        // Robot-centric displacement
        val robotDx = (dLeft + dRight) / 2.0
        val robotDy = dStrafe
        
        // Transform to field-centric using current heading
        pose.x += robotDx * cos(pose.heading) - robotDy * sin(pose.heading)
        pose.y += robotDx * sin(pose.heading) + robotDy * cos(pose.heading)
        pose.heading = (pose.heading + dHeadingNormalized) % (2 * PI)
        
        // Calculate velocities
        val dx = robotDx * cos(pose.heading) - robotDy * sin(pose.heading)
        val dy = robotDx * sin(pose.heading) + robotDy * cos(pose.heading)
        
        velocityX = velocityAlpha * dx + (1 - velocityAlpha) * velocityX
        velocityY = velocityAlpha * dy + (1 - velocityAlpha) * velocityY
        angularVelocity = velocityAlpha * dHeadingNormalized + (1 - velocityAlpha) * angularVelocity
        
        // Store previous values
        prevLeft = currentLeft
        prevRight = currentRight
        prevStrafe = currentStrafe
        prevHeading = currentHeading
    }
    
    /**
     * Reset pose to specific position and heading
     */
    fun resetPose(newPose: Pose) {
        pose = newPose
        prevLeft = leftEncoder.currentPosition.toDouble()
        prevRight = rightEncoder.currentPosition.toDouble()
        prevStrafe = strafeEncoder.currentPosition.toDouble()
        prevHeading = imu.robotHeading.angle.toRadians()
    }
    
    /**
     * Reset to origin facing forward
     */
    fun reset() {
        resetPose(Pose(0.0, 0.0, 0.0))
    }
    
    /**
     * Get velocity magnitude
     */
    fun getSpeed(): Double = sqrt(velocityX.pow(2) + velocityY.pow(2))
    
    /**
     * Get heading in degrees
     */
    fun getHeadingDegrees(): Double = Math.toDegrees(pose.heading)
}

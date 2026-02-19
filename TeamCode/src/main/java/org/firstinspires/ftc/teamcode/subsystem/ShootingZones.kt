package org.firstinspires.ftc.teamcode.subsystem

import kotlin.math.PI

/**
 * Field dimensions (FTC standard field: 12ft x 12ft = 144" x 144")
 */
object FieldConstants {
    const val FIELD_WIDTH = 144.0  // inches
    const val FIELD_HEIGHT = 144.0 // inches
    
    // Define your team's starting position
    val START_POSE = Pose(0.0, 0.0, 0.0) // adjust as needed
}

/**
 * Predefined shooting zones
 * Adjust coordinates to match your field setup
 */
object ShootingZones {
    
    /**
     * Example: Standard FTC shooting zone near the hub
     * Adjust vertices to match your auto line and desired shooting positions
     */
    val MAIN_ZONE = ShootingZone(listOf(
        Pose(0.0, 0.0),       // bottom-left corner of zone
        Pose(48.0, 0.0),      // bottom-right
        Pose(48.0, 36.0),     // top-right
        Pose(0.0, 36.0)       // top-left
    ))
    
    /**
     * Close range shooting zone (near hub)
     */
    val CLOSE_ZONE = ShootingZone(listOf(
        Pose(0.0, 0.0),
        Pose(24.0, 0.0),
        Pose(24.0, 24.0),
        Pose(0.0, 24.0)
    ))
    
    /**
     * Mid-range shooting zone
     */
    val MID_ZONE = ShootingZone(listOf(
        Pose(24.0, 0.0),
        Pose(48.0, 0.0),
        Pose(48.0, 24.0),
        Pose(24.0, 24.0)
    ))
    
    /**
     * Far shooting zone (auto line)
     */
    val FAR_ZONE = ShootingZone(listOf(
        Pose(48.0, 0.0),
        Pose(72.0, 0.0),
        Pose(72.0, 36.0),
        Pose(48.0, 36.0)
    ))
    
    /**
     * Get all zones as a list
     */
    val ALL_ZONES = listOf(CLOSE_ZONE, MID_ZONE, FAR_ZONE)
    
    /**
     * Find which zone the robot is in (if any)
     */
    fun getZone(pose: Pose): String {
        return when {
            CLOSE_ZONE.contains(pose) -> "CLOSE"
            MID_ZONE.contains(pose) -> "MID"
            FAR_ZONE.contains(pose) -> "FAR"
            MAIN_ZONE.contains(pose) -> "MAIN"
            else -> "NONE"
        }
    }
    
    /**
     * Get distance to closest zone center
     */
    fun distanceToZone(pose: Pose): Double {
        var minDist = Double.MAX_VALUE
        for (zone in ALL_ZONES) {
            val dist = zone.distanceTo(pose)
            minDist = min(minDist, dist)
        }
        return minDist
    }
}

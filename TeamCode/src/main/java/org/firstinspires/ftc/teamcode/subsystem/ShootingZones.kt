package org.firstinspires.ftc.teamcode.subsystem

/**
 * Field dimensions (FTC standard field: 12ft x 12ft = 144" x 144")
 */
object FieldConstants {
    const val FIELD_WIDTH = 144.0
    const val FIELD_HEIGHT = 144.0
    
    // Adjust to match your starting position
    val START_POSE = RobotPose(0.0, 0.0, 0.0)
}

/**
 * Predefined shooting zones
 * Adjust coordinates to match your field/hub position
 */
object ShootingZones {
    
    /**
     * Main shooting zone near the hub
     */
    val MAIN_ZONE = ShootingZone(listOf(
        RobotPose(0.0, 0.0),
        RobotPose(48.0, 0.0),
        RobotPose(48.0, 36.0),
        RobotPose(0.0, 36.0)
    ))
    
    /**
     * Close range (right against hub)
     */
    val CLOSE_ZONE = ShootingZone(listOf(
        RobotPose(0.0, 0.0),
        RobotPose(24.0, 0.0),
        RobotPose(24.0, 24.0),
        RobotPose(0.0, 24.0)
    ))
    
    /**
     * Mid-range
     */
    val MID_ZONE = ShootingZone(listOf(
        RobotPose(24.0, 0.0),
        RobotPose(48.0, 0.0),
        RobotPose(48.0, 24.0),
        RobotPose(24.0, 24.0)
    ))
    
    /**
     * Far range (auto line)
     */
    val FAR_ZONE = ShootingZone(listOf(
        RobotPose(48.0, 0.0),
        RobotPose(72.0, 0.0),
        RobotPose(72.0, 36.0),
        RobotPose(48.0, 36.0)
    ))
    
    val ALL_ZONES = listOf(CLOSE_ZONE, MID_ZONE, FAR_ZONE)
    
    /**
     * Get which zone robot is in
     */
    fun getZone(pose: RobotPose): String {
        return when {
            CLOSE_ZONE.contains(pose) -> "CLOSE"
            MID_ZONE.contains(pose) -> "MID"
            FAR_ZONE.contains(pose) -> "FAR"
            MAIN_ZONE.contains(pose) -> "MAIN"
            else -> "NONE"
        }
    }
    
    /**
     * Distance to nearest zone
     */
    fun distanceToZone(pose: RobotPose): Double {
        var minDist = Double.MAX_VALUE
        for (zone in ALL_ZONES) {
            minDist = min(minDist, zone.distanceTo(pose))
        }
        return minDist
    }
}

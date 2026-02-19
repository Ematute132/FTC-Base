package org.firstinspires.ftc.teamcode.subsystem

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx

/**
 * Hood subsystem for adjusting shooter angle.
 */
object Hood : Subsystem {
    private var hood = ServoEx("hood")
    
    private var targetPosition = 0.0
    
    override fun periodic() {
        hood.position = targetPosition
    }
    
    /**
     * Set hood position (0.0 to 1.0)
     */
    fun setPosition(newPosition: Double) {
        targetPosition = newPosition.coerceIn(0.0, 1.0)
    }
    
    /**
     * Get current position
     */
    fun getPosition(): Double = targetPosition
    
    // ==================== PRESETS ====================
    
    val full = InstantCommand { targetPosition = 1.0 }
    val close = InstantCommand { targetPosition = 0.0 }
    val mid = InstantCommand { targetPosition = 0.5 }
    val far = InstantCommand { targetPosition = 0.75 }
    
    /**
     * Set based on distance zone
     */
    fun setForZone(zone: String) {
        when (zone) {
            "CLOSE" -> setPosition(0.0)
            "MID" -> setPosition(0.5)
            "FAR" -> setPosition(0.75)
            else -> setPosition(0.5)
        }
    }
}

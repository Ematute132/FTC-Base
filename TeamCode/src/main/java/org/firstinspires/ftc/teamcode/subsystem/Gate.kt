package org.firstinspires.ftc.teamcode.subsystem

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx

/**
 * Gate subsystem for controlling ball flow to shooter.
 */
object Gate : Subsystem {
    private var servo = ServoEx("gate", 0.01)
    private var position = 0.0

    // State
    enum class State {
        OPEN, CLOSED
    }
    
    var currentState = State.CLOSED
        private set

    override fun periodic() {
        servo.position = position
    }

    // ==================== COMMANDS ====================
    val open = InstantCommand {
        position = 0.0
        currentState = State.OPEN
    }

    val close = InstantCommand {
        position = 1.0
        currentState = State.CLOSED
    }

    fun getState(): State = currentState
}

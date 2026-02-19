package org.firstinspires.ftc.teamcode.subsystem

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx

/**
 * Intake subsystem
 * Controls intake motor - intakes (positive) and outtakes (negative)
 */
object Intake : Subsystem {
    private var motor = MotorEx("intake")

    // Power settings
    var intakePower = 1.0
    var outtakePower = -1.0

    // State
    enum class State {
        INTAKING, OUTTAKING, STOPPED
    }
    
    var currentState = State.STOPPED
        private set

    override fun periodic() {
        // Motor updates automatically through MotorEx
    }

    // ==================== COMMANDS ====================
    val run = InstantCommand {
        motor.power = intakePower
        currentState = State.INTAKING
    }

    val outtake = InstantCommand {
        motor.power = outtakePower
        currentState = State.OUTTAKING
    }

    val stop = InstantCommand {
        motor.power = 0.0
        currentState = State.STOPPED
    }

    fun getState(): State = currentState
}

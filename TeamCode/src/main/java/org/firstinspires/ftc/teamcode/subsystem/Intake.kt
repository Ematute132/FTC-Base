package org.firstinspires.ftc.teamcode.subsystem

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower

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

    // Commands using SetPower
    var intake = SetPower(motor, intakePower).also { currentState = State.INTAKING }
    var reverse = SetPower(motor, outtakePower).also { currentState = State.OUTTAKING }
    var off = SetPower(motor, 0.0).also { currentState = State.STOPPED }

    fun getState(): State = currentState
}

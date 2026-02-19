package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import dev.nextftc.core.subsystems.Subsystem

/**
 * Gate subsystem
 * Controls gate servo - opens and closes to let balls through
 */
object Gate : Subsystem {
    private lateinit var servo: Servo

    // Positions
    const val POSITION_CLOSED = 0.0
    const val POSITION_OPEN = 1.0

    // State
    enum class State {
        OPEN, CLOSED
    }
    
    var currentState = State.CLOSED
        private set

    override fun initialize(hardwareMap: HardwareMap) {
        servo = hardwareMap.get(Servo::class.java, "gate")
        servo.direction = Servo.Direction.FORWARD
        
        // Start closed
        close()
    }

    fun open() {
        servo.position = POSITION_OPEN
        currentState = State.OPEN
    }

    fun close() {
        servo.position = POSITION_CLOSED
        currentState = State.CLOSED
    }

    fun toggle() {
        if (currentState == State.CLOSED) {
            open()
        } else {
            close()
        }
    }

    fun getState(): State = currentState
}

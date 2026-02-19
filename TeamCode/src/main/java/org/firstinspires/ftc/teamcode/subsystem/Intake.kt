package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.nextftc.core.subsystems.Subsystem

/**
 * Intake subsystem
 * Controls intake motor - intakes (positive) and outtakes (negative)
 */
object Intake : Subsystem {
    private lateinit var motor: DcMotor

    // Power settings
    var intakePower = 1.0
    var outtakePower = -1.0

    // State
    enum class State {
        INTAKING, OUTTAKING, STOPPED
    }
    
    var currentState = State.STOPPED
        private set

    override fun initialize(hardwareMap: HardwareMap) {
        motor = hardwareMap.get(DcMotor::class.java, "intake")
        motor.direction = DcMotor.Direction.FORWARD
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun run() {
        motor.power = intakePower
        currentState = State.INTAKING
    }

    fun outtake() {
        motor.power = outtakePower
        currentState = State.OUTTAKING
    }

    fun stop() {
        motor.power = 0.0
        currentState = State.STOPPED
    }

    fun getState(): State = currentState
}

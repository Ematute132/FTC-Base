package org.firstinspires.ftc.teamcode.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.VoltageSensor
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforward
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx

/**
 * Flywheel subsystem
 * Uses velocity PID + feedforward for consistent shooter speed
 */
@Configurable
object Flywheel : Subsystem {
    // Hardware
    private var motor = MotorEx("flywheel")

    // Voltage compensation
    private val battery: VoltageSensor by lazy {
        hardwareMap.get(VoltageSensor::class.java, "Control Hub")
    }
    private const val V_NOMINAL = 12.0
    private var voltFilt = 12.0
    private const val ALPHA_VOLT = 0.08
    @JvmField var voltageCompEnabled = true

    // ============================================
    // TUNABLE PID VALUES (via Panels)
    // ============================================
    @JvmField var kP = 0.02
    @JvmField var kI = 0.0
    @JvmField var kD = 0.0
    
    // Feedforward values
    @JvmField var kV = 0.1  // Velocity feedforward
    @JvmField var kA = 0.0  // Acceleration feedforward
    @JvmField var kS = 0.0  // Static friction

    // Target RPM
    @JvmField var targetRPM = 3000.0
    
    // Max power
    @JvmField var maxPower = 1.0

    // Control system
    private val controller = controlSystem {
        velocityPid(PIDCoefficients(kP, kI, kD))
        feedforward(BasicFeedforward(kV, kA, kS))
    }

    // State
    enum class State {
        RUNNING, STOPPED
    }
    
    var currentState = State.STOPPED
        private set

    // RPM conversion (depends on your motor/encoder)
    // Adjust TICKS_PER_REV for your setup
    private val TICKS_PER_REV = 537.7  // GoBilda Yellow Jacket
    private val MS_PER_MINUTE = 60000.0

    override fun periodic() {
        // Voltage compensation
        val voltRaw = battery.voltage.coerceAtLeast(9.0)
        voltFilt += ALPHA_VOLT * (voltRaw - voltFilt)
        val voltageRatio = V_NOMINAL / voltFilt

        // Get current velocity in RPM
        val currentVelocity = motor.velocity
        val currentRPM = (currentVelocity * MS_PER_MINUTE) / TICKS_PER_REV

        // Run PID controller if running
        if (currentState == State.RUNNING) {
            // Convert RPM to ticks per second for controller
            val targetTicksPerSec = (targetRPM * TICKS_PER_REV) / MS_PER_MINUTE
            
            controller.goal = KineticState(targetTicksPerSec)
            var power = controller.calculate(motor.state)
            
            // Apply voltage compensation
            if (voltageCompEnabled) {
                power *= voltageRatio
            }
            
            motor.power = power.coerceIn(-maxPower, maxPower)
        }
    }

    // ============================================
    // COMMANDS
    // ============================================
    fun setTargetRPM(rpm: Double) {
        targetRPM = rpm
        currentState = State.RUNNING
    }

    fun stop() {
        motor.power = 0.0
        currentState = State.STOPPED
    }

    fun getTargetRPM(): Double = targetRPM

    fun getCurrentRPM(): Double {
        val currentVelocity = motor.velocity
        return (currentVelocity * MS_PER_MINUTE) / TICKS_PER_REV
    }

    fun getState(): State = currentState
}

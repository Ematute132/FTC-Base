package org.firstinspires.ftc.teamcode.subsystem

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.hardware.VoltageSensor
import dev.nextftc.control.ControlSystem
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.ftc.ActiveOpMode
import java.util.function.Supplier

/**
 * Flywheel subsystem with voltage compensation
 */
@Configurable
object Flywheel : Subsystem {
    // Hardware - 2 Motors
    private val motor1 = MotorEx("Fly1").floatMode()
    private val motor2 = MotorEx("Fly2").floatMode()
    
    // Voltage sensor for compensation
    private val battery: VoltageSensor by lazy {
        ActiveOpMode.hardwareMap.get(VoltageSensor::class.java, "Control Hub")
    }
    
    // PID & FF Coefficients
    @JvmField var ffCoefficients = BasicFeedforwardParameters(0.003, 0.08, 0.0)
    @JvmField var pidCoefficients = PIDCoefficients(0.009, 0.0, 0.01)
    
    // Control System
    val controller: ControlSystem = controlSystem {
        basicFF(ffCoefficients)
        velPid(pidCoefficients)
    }
    
    // Voltage compensation constants
    private const val V_NOMINAL = 12.0
    private var voltFilt = 12.0
    private const val ALPHA_VOLT = 0.08
    
    // Voltage comp toggle
    @JvmField var voltageCompEnabled = true
    
    // Target velocity
    private var targetVelocity = 0.0
    
    // ==================== VELOCITY PRESETS ====================
    
    /**
     * Set flywheel velocity
     */
    fun setVelocity(speed: Double) {
        targetVelocity = speed
        controller.goal = KineticState(0.0, speed)
    }
    
    // Presets
    val off = InstantCommand { setVelocity(0.0) }
    val close = InstantCommand { setVelocity(1000.0) }
    val mid = InstantCommand { setVelocity(1250.0) }
    val far = InstantCommand { setVelocity(1500.0) }
    val max = InstantCommand { setVelocity(1500.0) }
    val maxAuto = InstantCommand { setVelocity(1600.0) }
    val idle = InstantCommand { setVelocity(-300.0) }
    val runHigh = InstantCommand { setVelocity(2000.0) }
    
    // ==================== MOTOR CONTROL ====================
    
    private fun setMotorPowers(power: Double) {
        val clampedPower = power.coerceIn(-0.85, 0.85)
        motor1.power = clampedPower
        motor2.power = clampedPower
    }
    
    // ==================== PERIODIC ====================
    
    override fun periodic() {
        // Get voltage and filter it
        val voltRaw = battery.voltage.coerceAtLeast(9.0)
        voltFilt += ALPHA_VOLT * (voltRaw - voltFilt)
        
        // Calculate voltage ratio
        val voltageRatio = V_NOMINAL / voltFilt
        
        // Calculate base power from controller
        val rawPower = controller.calculate(motor1.state)
        
        // Apply voltage compensation
        val finalPower = if (voltageCompEnabled) {
            (rawPower * voltageRatio).coerceIn(-0.85, 0.85)
        } else {
            rawPower.coerceIn(-0.85, 0.85)
        }
        
        setMotorPowers(finalPower)
        
        // Telemetry
        PanelsTelemetry.telemetry.addData("Flywheel Power", finalPower)
        PanelsTelemetry.telemetry.addData("Target Vel", targetVelocity)
        PanelsTelemetry.telemetry.addData("Actual Vel", motor1.velocity)
        PanelsTelemetry.telemetry.addData("Voltage", voltFilt)
        PanelsTelemetry.telemetry.addData("Voltage Comp", voltageRatio)
    }
    
    // ==================== COMMANDS ====================
    
    /**
     * Manual control - override with direct power
     */
    class Manual(private val shooterPower: Supplier<Double>) : Command() {
        override val isDone = false
        init { requires(Flywheel) }
        override fun update() {
            Flywheel.setMotorPowers(shooterPower.get())
        }
    }
    
    /**
     * Check if at target velocity
     */
    fun isAtTarget(): Boolean {
        return ((targetVelocity - 20.0) < motor1.velocity) && 
               ((targetVelocity + 40.0) > motor1.velocity)
    }
    
    /**
     * Get current velocity
     */
    fun getVelocity(): Double = motor1.velocity
    
    /**
     * Get target velocity
     */
    fun getTargetVelocity(): Double = targetVelocity
}

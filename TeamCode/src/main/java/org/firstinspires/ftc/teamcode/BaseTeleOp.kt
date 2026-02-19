package org.firstinspires.ftc.teamcode

import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.hardware.driving.DriverControlledCommand
import org.firstinspires.ftc.teamcode.subsystem.Gate
import org.firstinspires.ftc.teamcode.subsystem.Intake

/**
 * Base TeleOp for FTC Robot
 * Uses NextFTC framework with Pedro Pathing integration
 * Panels telemetry enabled for live debugging
 */
@TeleOp(name = "Base TeleOp", group = "TeleOp")
class BaseTeleOp : NextFTCOpMode() {

    // Panels telemetry for live dashboard
    private val panelsTelemetry: TelemetryManager by lazy { 
        TelemetryManager() 
    }

    // Pedro Pathing follower
    private val follower by lazy { PedroComponent.follower }

    // Drivetrain command
    private val drivetrain: DriverControlledCommand by lazy {
        PedroDriverControlled(
            -Gamepads.gamepad1.leftStickY,
            -Gamepads.gamepad1.leftStickX,
            -Gamepads.gamepad1.rightStickX,
            true // useFieldCentric
        )
    }

    override fun onInit() {
        // Initialize subsystems
        Gate.initialize(hardwareMap)
        Intake.initialize(hardwareMap)

        // Initialize Pedro Pathing - set starting pose
        follower.setStartingPose(Pose(0.0, 0.0, 0.0))

        // Send init message to panels
        panelsTelemetry.addLine("BaseTeleOp Initialized")
    }

    override fun onStartButtonPressed() {
        // Start drivetrain
        drivetrain.schedule()

        // Gamepad controls
        setupGamepadControls()
    }

    private fun setupGamepadControls() {
        // Drivetrain
        // Slow mode with left bumper
        Gamepads.gamepad1.leftBumper
            .toggleOnBecomesTrue()
            .whenBecomesTrue { drivetrain.scalar = 0.5 }
            .whenBecomesFalse { drivetrain.scalar = 1.0 }

        // Intake control with triggers
        // Right trigger = intake
        Gamepads.gamepad1.rightTrigger.greaterThan(0.0)
            .whenBecomesTrue { Intake.run() }
            .whenBecomesFalse { Intake.stop() }

        // Left trigger = outtake
        Gamepads.gamepad1.leftTrigger.greaterThan(0.0)
            .whenBecomesTrue { Intake.outtake() }
            .whenBecomesFalse { Intake.stop() }

        // Right bumper = toggle gate
        Gamepads.gamepad1.rightBumper
            .whenBecomesTrue { Gate.toggle() }
    }

    override fun onUpdate() {
        // Update Pedro Pathing follower
        follower.update()

        // Send telemetry to both Driver Station and Panels
        updateTelemetry()
    }

    private fun updateTelemetry() {
        // Robot position
        panelsTelemetry.addLine("X: %.1f".format(follower.pose.x))
        panelsTelemetry.addLine("Y: %.1f".format(follower.pose.y))
        panelsTelemetry.addLine("Heading: %.1f".format(Math.toDegrees(follower.pose.heading)))

        // Subsystems
        panelsTelemetry.addLine("Intake: ${Intake.getState()}")
        panelsTelemetry.addLine("Gate: ${Gate.getState()}")

        // Update both Driver Station and Panels
        panelsTelemetry.push(telemetry)
    }
}

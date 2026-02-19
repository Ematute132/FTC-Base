package org.firstinspires.ftc.teamcode

import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.hardware.driving.DriverControlledCommand

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

    // Robot subsystems - add your subsystems here
    // private lateinit var drive: Drive
    // private lateinit var intake: Intake
    // private lateinit var shooter: Shooter

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
        // Initialize subsystems here
        // drive = Drive(hardwareMap)
        // intake = Intake(hardwareMap)
        // shooter = Shooter(hardwareMap)

        // Initialize Pedro Pathing - set starting pose
        follower.setStartingPose(Pose(0.0, 0.0, 0.0))

        // Send init message to panels
        panelsTelemetry.addLine("BaseTeleOp Initialized")
    }

    override fun onStartButtonPressed() {
        // Start drivetrain
        drivetrain.schedule()

        // Gamepad controls - customize for your robot
        setupGamepadControls()
    }

    private fun setupGamepadControls() {
        // Example controls - modify for your robot

        // Gamepad 1 - Drivetrain (already scheduled above)
        // Use gamepad1.leftStickX/Y for movement
        // Use gamepad1.rightStickX for rotation

        // Slow mode with bumper
        Gamepads.gamepad1.leftBumper
            .toggleOnBecomesTrue()
            .whenBecomesTrue { drivetrain.scalar = 0.5 }
            .whenBecomesFalse { drivetrain.scalar = 1.0 }

        // Gamepad 2 - Mechanisms
        // Uncomment and customize:
        // Gamepads.gamepad2.a.whenBecomesTrue { intake.run() }
        // Gamepads.gamepad2.b.whenBecomesTrue { shooter.shoot() }
        // Gamepads.gamepad2.x.whenBecomesTrue { shooter.stop() }
    }

    override fun onUpdate() {
        // Update Pedro Pathing follower
        follower.update()

        // Update subsystems
        // drive.update()
        // intake.update()
        // shooter.update()

        // Send telemetry to both Driver Station and Panels
        updateTelemetry()
    }

    private fun updateTelemetry() {
        // Robot position
        panelsTelemetry.addLine("X: %.1f".format(follower.pose.x))
        panelsTelemetry.addLine("Y: %.1f".format(follower.pose.y))
        panelsTelemetry.addLine("Heading: %.1f".format(Math.toDegrees(follower.pose.heading)))

        // Debug info - customize for your robot
        // panelsTelemetry.addLine("Intake: ${intake.state}")
        // panelsTelemetry.addLine("Shooter: ${shooter.state}")

        // Update both Driver Station and Panels
        panelsTelemetry.push(telemetry)
    }
}

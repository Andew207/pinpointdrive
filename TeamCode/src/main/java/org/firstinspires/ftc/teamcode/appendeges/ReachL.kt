package org.firstinspires.ftc.teamcode.appendeges

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap


class ReachL(hardwareMap: HardwareMap) {

    /**
     * @param position the position of the scoringArm in that state, -1 means we don't currently
     * know the position of the scoringArm
     */
    enum class ReachLState(val position: Int) {
        In(0),
        Out(2900)
    }

    var reachLState = ReachLState.In

    private val reachL = hardwareMap.get(DcMotor::class.java, "inOutLeft")

    private val power = 0.75

    var reachLOffset = 0 //offset used to reset the arm positions mid-match
    var targetPosition = 0.0

    init {
        reachL.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        reachL.direction = DcMotorSimple.Direction.REVERSE
        reachL.targetPosition = 0
        reachL.mode = DcMotor.RunMode.RUN_TO_POSITION
        reachL.power = power
    }

    /**
     * Start: sets the arm state and position;
     * IsFinished: the arm has reached the state's position
     *
     * @param state the state (and associated position) to set the arm to
     */
    inner class SetState(private val state: ReachLState) : Action {
        private var initialized = false

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                targetPosition = state.position.toDouble()
                reachL.targetPosition = targetPosition.toInt() - reachLOffset
                reachLState = state
                initialized = true
            }
            reachL.currentPosition
            packet.put("Target Position", reachL.targetPosition)
            packet.put("Current Position", reachL.currentPosition)
            //TODO: make this scoringArm.isbusy so that it will actually do smth :)
            return false
        }
    }

    /**
     * manually changes the position of the scoringArm (typically with a joystick)
     *
     * @param input the percent speed (-1 to 1) normalized by delta time (the time between each loop)
     */

    /**
     * Only use in the collect position; used to reset the positions of the arm; should be called
     * alongside a collect action
     */

    fun inn(): Action = SetState(ReachLState.In)
    fun out(): Action = SetState(ReachLState.Out)


}
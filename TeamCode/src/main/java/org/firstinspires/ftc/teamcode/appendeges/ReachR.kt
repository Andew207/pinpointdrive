package org.firstinspires.ftc.teamcode.appendeges

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap


class ReachR(hardwareMap: HardwareMap) {

    /**
     * @param position the position of the scoringArm in that state, -1 means we don't currently
     * know the position of the scoringArm
     */
    enum class ReachRState(val position: Int) {
        In(0),
        Out(2900)
    }

    var reachRState = ReachRState.In

    private val reachR = hardwareMap.get(DcMotor::class.java, "inOutRight")

    private val power = 0.75

    var reachROffset = 0
    var targetPosition = 0.0

    init {
        reachR.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        reachR.direction = DcMotorSimple.Direction.REVERSE
        reachR.targetPosition = 0
        reachR.mode = DcMotor.RunMode.RUN_TO_POSITION
        reachR.power = power
    }

    /**
     * Start: sets the arm state and position;
     * IsFinished: the arm has reached the state's position
     *
     * @param state the state (and associated position) to set the arm to
     */
    inner class SetState(private val state: ReachRState) : Action {
        private var initialized = false

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                targetPosition = state.position.toDouble()
                reachR.targetPosition = targetPosition.toInt() - reachROffset
                reachRState = state
                initialized = true
            }
            reachR.currentPosition
            packet.put("Target Position", reachR.targetPosition)
            packet.put("Current Position", reachR.currentPosition)
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

    fun inn(): Action = SetState(ReachRState.In)
    fun out(): Action = SetState(ReachRState.Out)


}
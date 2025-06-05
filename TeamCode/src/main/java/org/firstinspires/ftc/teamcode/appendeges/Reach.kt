package org.firstinspires.ftc.teamcode.appendeges

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap


class Reach(hardwareMap: HardwareMap) {

    /**
     * @param position the position of the scoringArm in that state, -1 means we don't currently
     * know the position of the scoringArm
     */
    enum class ReachState(val position: Int) {
        Inn(0),
        Little(500),
        Middle(1030),
        Out(1800)
    }

    var reachState = ReachState.Inn

    private val reachL = hardwareMap.get(DcMotor::class.java, "inOutLeft")
    private val reachR = hardwareMap.get(DcMotor::class.java, "inOutRight")


    private val power = 1.0

    var reachOffset = 0 //offset used to reset the arm positions mid-match
    var targetPosition = 0.0

    init {
        reachL.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        reachL.direction = DcMotorSimple.Direction.REVERSE
        reachL.targetPosition = 0
        reachL.mode = DcMotor.RunMode.RUN_TO_POSITION
        reachL.power = power
        reachR.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        reachR.direction = DcMotorSimple.Direction.FORWARD
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
    inner class SetState(private val state: ReachState) : Action {
        private var initialized = false

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                targetPosition = state.position.toDouble()
                reachL.targetPosition = targetPosition.toInt() - reachOffset
                reachR.targetPosition = targetPosition.toInt() - reachOffset
                reachState = state
                initialized = true
            }
            reachL.currentPosition
            packet.put("Target Position", reachL.targetPosition)
            packet.put("Current Position", reachL.currentPosition)
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

    fun inn(): Action = SetState(ReachState.Inn)
    fun a_little(): Action = SetState(ReachState.Little)
    fun middle(): Action = SetState(ReachState.Middle)
    fun out(): Action = SetState(ReachState.Out)


}
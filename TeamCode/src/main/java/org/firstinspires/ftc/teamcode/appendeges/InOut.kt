package org.firstinspires.ftc.teamcode.appendeges

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap


class InOut(hardwareMap: HardwareMap) {

    /**
     * @param position the position of the scoringArm in that state, -1 means we don't currently
     * know the position of the scoringArm
     */
    enum class InOutState(val position: Int) {
        In(0), // lol funny number
        Out(1000)
    }

    var armState = InOutState.In

    private val inOutLeft = hardwareMap.get(DcMotor::class.java, "inOutLeft")
    private val inOutRight = hardwareMap.get(DcMotor::class.java, "inOutRight")

    private val power = 0.75

    var scoringArmOffset = 0 //offset used to reset the arm positions mid-match
    var targetPosition = 0.0

    init {
        inOutLeft.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        inOutRight.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        inOutLeft.direction = DcMotorSimple.Direction.FORWARD
        inOutRight.direction = DcMotorSimple.Direction.REVERSE
        inOutLeft.targetPosition = 0
        inOutRight.targetPosition = 0
        inOutLeft.mode = DcMotor.RunMode.RUN_TO_POSITION
        inOutRight.mode = DcMotor.RunMode.RUN_TO_POSITION
        inOutLeft.power = power
        inOutRight.power = power
    }

    /**
     * Start: sets the arm state and position;
     * IsFinished: the arm has reached the state's position
     *
     * @param state the state (and associated position) to set the arm to
     */
    inner class SetState(private val state: InOutState) : Action {
        private var initialized = false

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                targetPosition = state.position.toDouble()
                inOutLeft.targetPosition = targetPosition.toInt() - scoringArmOffset
                inOutRight.targetPosition = targetPosition.toInt() - scoringArmOffset
                armState = state
                initialized = true
            }
            inOutLeft.currentPosition
            inOutRight.currentPosition
            packet.put("Target Position", inOutLeft.targetPosition)
            packet.put("Current Position", inOutLeft.currentPosition)
            packet.put("Target Position", inOutRight.targetPosition)
            packet.put("Current Position", inOutRight.currentPosition)
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


}
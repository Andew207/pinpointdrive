package org.firstinspires.ftc.teamcode.appendeges

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap


class ArmSwing(hardwareMap: HardwareMap) {

    /**
     * @param position the position of the scoringArm in that state, -1 means we don't currently
     * know the position of the scoringArm
     */
    enum class ArmState(val position: Int) {
        Score1(1900),
        Score2(1750), // lol funny number
        ThroughBars2(1290),
        ThroughBars1(1070),
        CornerPickup(250),
        Neutral(400),

        PickUp(100)

    }

    var armState = ArmState.Neutral

    private val armSwing = hardwareMap.get(DcMotor::class.java, "armSwing")

    private val power = 1.0

    var scoringArmOffset = 0 //offset used to reset the arm positions mid-match
    var targetPosition = 0.0

    init {
        armSwing.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        armSwing.direction = DcMotorSimple.Direction.FORWARD
        armSwing.targetPosition = 0
        armSwing.mode = DcMotor.RunMode.RUN_TO_POSITION
        armSwing.power = power
    }

    /**
     * Start: sets the arm state and position;
     * IsFinished: the arm has reached the state's position
     *
     * @param state the state (and associated position) to set the arm to
     */
    inner class SetState(private val state: ArmState) : Action {
        private var initialized = false

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                targetPosition = state.position.toDouble()
                armSwing.targetPosition = targetPosition.toInt() - scoringArmOffset
                armState = state
                initialized = true
            }
            armSwing.currentPosition
            packet.put("Target Position", armSwing.targetPosition)
            packet.put("Current Position", armSwing.currentPosition)
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

    fun throughBars1(): Action = SetState(ArmState.ThroughBars1)
    fun throughBars2(): Action = SetState(ArmState.ThroughBars2)
    fun neutral(): Action = SetState(ArmState.Neutral)
    fun corner(): Action = SetState(ArmState.CornerPickup)
    fun pickup(): Action = SetState(ArmState.PickUp)
    fun score1(): Action = SetState(ArmState.Score1)
    fun score2(): Action = SetState(ArmState.Score2)

}
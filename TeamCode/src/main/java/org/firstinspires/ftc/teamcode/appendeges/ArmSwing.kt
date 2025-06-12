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
        Score2(2425),
        ThroughBars2(1700),
        ThroughBars3(1480),
        ThroughBars1(1480),
        ThroughBars0(1360),
        ThroughBars4(1200),
        ThroughBars5(1120),
        CornerPickup(250),
        Neutral(500),
        Wall(665),
        Wall2(620),
        PickUp(200),
        Sweep(50),
        Init(-150)

    }

    var armState = ArmState.Neutral

    private val armSwing = hardwareMap.get(DcMotor::class.java, "armSwing")


    private val power = 1.0

    var scoringArmOffset = 0 //offset used to reset the arm positions mid-match
    var targetPosition = 0.0

    init {
        armSwing.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        armSwing.direction = DcMotorSimple.Direction.FORWARD
        armSwing.targetPosition = -150
        armSwing.mode = DcMotor.RunMode.RUN_TO_POSITION
        armSwing.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
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

    fun throughBars5(): Action = SetState(ArmState.ThroughBars5)
    fun throughBars4(): Action = SetState(ArmState.ThroughBars4)
    fun throughBars0(): Action = SetState(ArmState.ThroughBars0)
    fun throughBars1(): Action = SetState(ArmState.ThroughBars1)
    fun throughBars2(): Action = SetState(ArmState.ThroughBars2)
    fun throughBars3(): Action = SetState(ArmState.ThroughBars3)
    fun neutral(): Action = SetState(ArmState.Neutral)
    fun corner(): Action = SetState(ArmState.CornerPickup)
    fun pickup(): Action = SetState(ArmState.PickUp)
    fun sweep(): Action = SetState(ArmState.Sweep)
    fun score1(): Action = SetState(ArmState.Score1)
    fun score2(): Action = SetState(ArmState.Score2)
    fun wall(): Action = SetState(ArmState.Wall)
    fun wall2(): Action = SetState(ArmState.Wall2)
    fun init(): Action = SetState(ArmState.Init)

}
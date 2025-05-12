package org.firstinspires.ftc.teamcode.appendeges

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo


class Wrist(hardwareMap: HardwareMap) {

    /**
     * @param position the position of the scoringArm in that state, -1 means we don't currently
     * know the position of the scoringArm
     */
    enum class WristPos(val position: Double) {
        Offset(0.9),
        AutoL4thblock(0.7), //TODO: Test
        Straight(0.6),
        Back(0.4),
        Score(0.2)
    }


    private val wrist = hardwareMap.get(Servo::class.java, "wrist")




    var targetPosition = 0.0


    /**
     * Start: sets the arm state and position;
     * IsFinished: the arm has reached the state's position
     *
     * @param state the state (and associated position) to set the arm to
     */
    inner class SetState(private val state: WristPos) : Action {
        private var initialized = false

        @Suppress("PARAMETER_NAME_CHANGED_ON_OVERRIDE")
        override fun run(packet: TelemetryPacket): Boolean {
            if (!initialized) {
                targetPosition = state.position.toDouble()
                wrist.position = targetPosition

                initialized = true
            }
            wrist.position

            //TODO: make this scoringArm.isbusy so that it will actually do smth :)
            return false
        }
    }

    /**
     * manually changes the position of the scoringArm (typically with a joystick)
     * @param input the percent speed (-1 to 1) normalized by delta time (the time between each loop)
     **/

    /**
     * Only use in the collect position; used to reset the positions of the arm; should be called
     * alongside a collect action
     */
    //TODO: change functions to wrist functions
    fun offset(): Action = SetState(WristPos.Offset)
    fun straight(): Action = SetState(WristPos.Straight)
    fun back(): Action = SetState(WristPos.Back)
    fun score(): Action = SetState(WristPos.Score)
    fun block4(): Action = SetState(WristPos.AutoL4thblock)

}
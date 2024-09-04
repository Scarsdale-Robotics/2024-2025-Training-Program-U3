package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;

public class RobotSystem {

    public final HardwareRobot HARDWARE_ROBOT;
    public final CVSubsystem CV;

    public RobotSystem(HardwareMap hardwareMap, LinearOpMode opMode) {
        HARDWARE_ROBOT = new HardwareRobot(hardwareMap);
        CV = new CVSubsystem(HARDWARE_ROBOT.cameraName, opMode);
    }

}

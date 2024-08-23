package org.firstinspires.ftc.teamcode.opmodes.autons;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.CVSubsystem;

public class AutonUsingCV extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // RobotSystem packages our subsystems
        RobotSystem robot = new RobotSystem(hardwareMap);

        CVSubsystem cv = robot.CV;

        waitForStart();

        while (opModeIsActive()) {
            int detectedOrangePixels = cv.getDetectedOrangePixels();

            // telemetry ~ standard output (python: print, java: System.out, c++: cout, js/ts: console.log)
            telemetry.addData("Detected Orange Pixels: ", detectedOrangePixels);
            telemetry.update();
        }

    }

}

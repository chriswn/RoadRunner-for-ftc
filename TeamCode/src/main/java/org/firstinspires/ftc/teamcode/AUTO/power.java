package org.firstinspires.ftc.teamcode.AUTO;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="power")
public class power extends LinearOpMode {

        // Declare the motor
        private DcMotor rightFront;

        @Override
        public void runOpMode() {
            // Initialize the motor
            rightFront = hardwareMap.get(DcMotor.class, "rightFront"); //  motor's name

            // Wait for the start command
            waitForStart();

            // Test the motor by running it at full power for a few seconds
            if (opModeIsActive()) {
                rightFront.setPower(1.0); // Full power forward
                sleep(2000); // Run for 2 seconds

                rightFront.setPower(-1.0); // Full power backward
                sleep(2000); // Run for 2 seconds

                rightFront.setPower(0); // Stop the motor
            }
        }
    }


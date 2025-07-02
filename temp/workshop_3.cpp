
#include "mbed.h"
#include "Servo.h"
#include "FastPWM.h"
#include "DCMotor.h"
#include "UltrasonicSensor.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include <IRSensor.h>

// float ir_sensor_compensation(float ir_distance_mV);

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1);   // create DebounceIn to evaluate the user button
void toggle_do_execute_main_fcn(); // custom function which is getting executed when user
                                   // button gets pressed, definition at the end




// main runs as an own thread
int main()
{
    // // set up states for state machine
    enum RobotState {
        INITIAL,
        SLEEP,
        FORWARD,
        BACKWARD,
        EMERGENCY
    } robot_state = RobotState::INITIAL;
    // attach button fall function address to user button object
    user_button.fall(&toggle_do_execute_main_fcn);

    // while loop gets executed every main_task_period_ms milliseconds, this is a
    // simple approach to repeatedly execute main
    const int main_task_period_ms = 20; // define main task period time in ms e.g. 20 ms, therefore
                                        // the main task will run 50 times per second
    Timer main_task_timer;              // create Timer object which we use to run the main task
                                        // every main_task_period_ms

    // led on nucleo board
    DigitalOut user_led(LED1);

    // mechanical button
    DigitalIn mechanical_button(PC_5); // create DigitalIn object to evaluate mechanical button, you
                                    // need to specify the mode for proper usage, see below
    mechanical_button.mode(PullUp);    // sets pullup between pin and 3.3 V, so that there
                                    // is a defined potential

    // additional led
    // create DigitalOut object to command extra led, you need to add an additional resistor, e.g. 220...500 Ohm
    // a led has an anode (+) and a cathode (-), the cathode needs to be connected to ground via the resistor
    DigitalOut led1(PB_9);

    // ultra sonic sensor
    UltrasonicSensor us_sensor(PB_D3);
    float us_distance_cm = 0.0f;

    // create object to enable power electronics for the DC motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    // motor M1
    // FastPWM pwm_M1(PB_PWM_M1); // create FastPWM object to command motor M1

    // motor M2
    // FastPWM pwm_M2(PB_PWM_M2); // create FastPWM object to command motor M2

    const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
    // //                              // 6.0f V if you only use one battery pack

    // // motor M2
    // const float gear_ratio_M1 = 391.0f; // gear ratio
    // const float kn_M1 = 36.0f / 12.0f;  // motor constant [rpm/V]
    // // it is assumed that only one motor is available, therefore
    // // we use the pins from M1, so you can leave it connected to M1
    // DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio_M1, kn_M1, voltage_max);

    // // limit max. acceleration to half of the default acceleration
    // motor_M1.setMaxAcceleration(motor_M1.getMaxAcceleration() * 0.5f);

    // // enable the motion planner for smooth movements
    // motor_M1.enableMotionPlanner();

    // motor M3
    const float gear_ratio_M3 = 78.0f; // gear ratio
    const float kn_M3 = 180.0f / 12.0f;  // motor constant [rpm/V]
    // it is assumed that only one motor is available, therefore
    // we use the pins from M1, so you can leave it connected to M1
    DCMotor motor_M3(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio_M3, kn_M3, voltage_max);
    // enable the motion planner for smooth movement
    motor_M3.enableMotionPlanner();
    // // limit max. velocity to half physical possible velocity
    // motor_M3.setMaxVelocity(motor_M3.getMaxPhysicalVelocity() * 0.5f);
    // motor_M3.setRotation(3.0f);
    // limit max. acceleration to half of the default acceleration
    motor_M3.setMaxAcceleration(motor_M3.getMaxAcceleration() * 0.5f);


    // // servo
    // Servo servo_D0(PB_D0);
    // Servo servo_D1(PB_D1);

    // // minimal pulse width and maximal pulse width obtained from the servo calibration process
    // // futaba S3001
    // float servo_D0_ang_min = 0.0150f; // careful, these values might differ from servo to servo
    // float servo_D0_ang_max = 0.1150f;
    // // reely S3003
    // float servo_D1_ang_min = 0.0250f;
    // float servo_D1_ang_max = 0.1300f;

    // // servo.setPulseWidth: before calibration (0,1) -> (min pwm, max pwm)
    // // servo.setPulseWidth: after calibration (0,1) -> (servo_D0_ang_min, servo_D0_ang_max)
    // servo_D0.calibratePulseMinMax(servo_D0_ang_min, servo_D0_ang_max);
    // servo_D1.calibratePulseMinMax(servo_D1_ang_min, servo_D1_ang_max);

    // // default acceleration of the servo motion profile is 1.0e6f
    // servo_D0.setMaxAcceleration(0.9f);

    // float servo_input = 0.0f;
    // // int servo_counter = 0; // define servo counter, this is an additional variable
    // //                     // used to command the servo
    // // const int loops_per_seconds = static_cast<int>(ceilf(1.0f / (0.001f * static_cast<float>(main_task_period_ms))));

    // // ir distance sensor
    // float ir_distance_mV = 0.0f; // define a variable to store measurement (in mV)
    float ir_distance_cm = 0.0f;
    // AnalogIn ir_analog_in(PC_2); // create AnalogIn object to read in the infrared distance sensor
    //                             // 0...3.3V are mapped to 0...1
    IRSensor ir_sensor(PC_2, 10309.7644f, -172.5231f); 

    // // min and max IR sensor reading, (us_distance_min_cm, us_distance_max_cm) -> (servo_min, servo_max)
    // float us_distance_min_cm = 5.0f;
    // float us_distance_max_cm = 13.0f;
    
    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        // read us sensor distance, only valid measurements will update us_distance_cm
        const float us_distance_cm_candidate = us_sensor.read();
        if (us_distance_cm_candidate > 0.0f)
            us_distance_cm = us_distance_cm_candidate;

        if (do_execute_main_task) {

            // // read analog input
            // ir_distance_mV = 1.0e3f * ir_analog_in.read() * 3.3f;
            // ir_distance_cm = ir_sensor_compensation(ir_distance_mV);
            // // ir_distance_mV = ir_sensor.readmV(); // sensor value in millivolts
            ir_distance_cm = ir_sensor.readcm(); // sensor value in centimeters (if calibrated) 

            // us_distance_cm = ir_distance_cm;
            // visual feedback that the main task is executed, setting this once would actually be enough
            led1 = 1;

            // enable hardwaredriver DC motors: 0 -> disabled, 1 -> enabled
            enable_motors = 1;

            // motor_M1.setVelocity(0.1f);

            // limit max. velocity to half physical possible velocity
            // motor_M1.setMaxVelocity(motor_M1.getMaxPhysicalVelocity() * 0.8f);
            // pwm_M1.write(0.75f); // apply 6V to the motor
            // pwm_M2.write(0.75f); // apply 6V to the motor

            // // enable the servos
            // if (!servo_D0.isEnabled())
            //     servo_D0.enable(0.5f);
            // if (!servo_D1.isEnabled())
            //     servo_D1.enable();

            // // command the servos
            // servo_D0.setPulseWidth(servo_input+0.5f);
            // servo_D1.setPulseWidth(servo_input);

            // // calculate inputs for the servos for the next cycle
            // if ((servo_input < 1.0f) &&                     // constrain servo_input to be < 1.0f
            //     (servo_counter % loops_per_seconds == 0) && // true if servo_counter is a multiple of loops_per_second
            //     (servo_counter != 0))                       // avoid servo_counter = 0
            //     servo_input += 0.005f;
            // servo_counter++;

            // // state machine
            switch (robot_state) {
                case RobotState::INITIAL: {
                    printf("INITIAL\n");
                    
                    enable_motors = 1;
                    robot_state = RobotState::SLEEP;

                    break;
                }

                case RobotState::SLEEP: {
                    printf("SLEEP\n");

                // wait for the signal from the user, so to run the process 
                // that is triggered by clicking the mechanical button
                // then go to the FORWARD state
                if (mechanical_button.read())
                    robot_state = RobotState::FORWARD;

                break;
                }
                case RobotState::FORWARD: {
                    // press is moving forward until it reaches 2.9f rotations, 
                    // when reaching the value go to BACKWARD
                    motor_M3.setRotation(2.9f); // setting this once would actually be enough
                    // if the distance from the sensor is less than 4.5f cm,
                    // we transition to the EMERGENCY state
                    if (us_distance_cm < 4.5f)
                        robot_state = RobotState::EMERGENCY;
                    // switching condition is slightly smaller for robustness
                    if (motor_M3.getRotation() > 2.89f)
                        robot_state = RobotState::BACKWARD;

                break;
            }
            case RobotState::BACKWARD: {
                    // move backwards to the initial position
                    // and go to the SLEEP state if reached
                    motor_M3.setRotation(0.0f);
                    // switching condition is slightly bigger for robustness
                    if (motor_M3.getRotation() < 0.01f)
                        robot_state = RobotState::SLEEP;

                    break;
            }
            case RobotState::EMERGENCY: {
                // disable the motion planner and
                // move to the initial position asap
                // then reset the system
                motor_M3.disableMotionPlanner();
                motor_M3.setRotation(0.0f);
                if (motor_M3.getRotation() < 0.01f)
                    toggle_do_execute_main_fcn();

                break;
            }
                default: {

                    break; // do nothing
                }
            }

        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // motor_M1.setVelocity(-0.1f);

                // enable hardwaredriver DC motors: 0 -> disabled, 1 -> enabled
                // enable_motors = 0;
                // reset variables and objects
                // reset variables and objects
                led1 = 0;
                enable_motors = 0;
                us_distance_cm = 0.0f;
                motor_M3.setMotionPlanerPosition(0.0f);
                motor_M3.setMotionPlanerVelocity(0.0f);
                motor_M3.enableMotionPlanner();
                robot_state = RobotState::INITIAL;
            }
        }

        // // print to the serial terminal
        // printf("IR distance mV: %f \n", ir_distance_mV);
        // print to the serial terminal
        // printf("IR distance mV: %f IR distance cm: %f \n", ir_distance_mV, ir_distance_cm);
        // printf("IR distance cm: %f \n", ir_distance_cm);

        // print to the serial terminal
        // printf("Motor velocity: %f \n", motor_M1.getVelocity());

        // print to the serial terminal
        // printf("Motor position: %f \n", motor_M3.getRotation());
        // print to the serial terminal
        printf("US Sensor in cm: %f, DC Motor Rotations: %f\n", us_distance_cm, motor_M3.getRotation());

        // print to the serial terminal
        // printf("Pulse width: %f \n", servo_input);
        // toggling the user led
        user_led = !user_led;

        // read timer and make the main thread sleep for the remaining time span (non blocking)
        int main_task_elapsed_time_ms = duration_cast<milliseconds>(main_task_timer.elapsed_time()).count();
        if (main_task_period_ms - main_task_elapsed_time_ms < 0)
            printf("Warning: Main task took longer than main_task_period_ms\n");
        else
            thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

void toggle_do_execute_main_fcn()
{
    // toggle do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
    // set do_reset_all_once to true if do_execute_main_task changed from false to true
    if (do_execute_main_task)
        do_reset_all_once = true;
}

float ir_sensor_compensation(float ir_distance_mV)
{
    // insert values that you got from the MATLAB file
    static const float a = 10309.7644f; // 10309.7644f, -172.5231f
    static const float b = -172.5231f;

    // avoid division by zero by adding a small value to the denominator
    if (ir_distance_mV + b == 0.0f)
        ir_distance_mV -= 0.001f;

    // printf("Print.. %f \n", ir_distance_mV + b);
    return a / (ir_distance_mV + b);
}


#include "mbed.h"
#include "Servo.h"
#include "FastPWM.h"
#include "DCMotor.h"
#include "UltrasonicSensor.h"
#include <Eigen/Dense>
#include "SensorBar.h"
#include "LineFollower.h"


#define M_PIf 3.14159265358979323846f // pi

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
    // attach button fall function address to user button object
    user_button.fall(&toggle_do_execute_main_fcn);

    // while loop gets executed every main_task_period_ms milliseconds, this is a
    // simple approach to repeatedly execute main
    const int main_task_period_ms = 20; // define main task period time in ms e.g. 20 ms, therefore
                                        // the main task will run 50 times per second
    Timer main_task_timer;              // create Timer object which we use to run the main task
                                        // every main_task_period_ms

    // sensor bar
    const float bar_dist = 0.114f; // distance from wheel axis to leds on sensor bar / array in meters
    // SensorBar sensor_bar(PB_9, PB_8, bar_dist);


    // angle measured from sensor bar (black line) relative to robot
    float angle{0.0f};

    // led on nucleo board
    // DigitalOut user_led(LED1);

    // additional led
    // create DigitalOut object to command extra led, you need to add an additional resistor, e.g. 220...500 Ohm
    // a led has an anode (+) and a cathode (-), the cathode needs to be connected to ground via the resistor
    // DigitalOut led1(PB_9);


    // create object to enable power electronics for the DC motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
    // //                              // 6.0f V if you only use one battery pack

    // motor 
    const float gear_ratio = 100.0f; // gear ratio
    const float kn = 140.0f / 12.0f;  // motor constant [rpm/V]
    // it is assumed that only one motor is available, therefore
    // we use the pins from M1, so you can leave it connected to M1
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);

    // // differential drive robot kinematics
    const float r_wheel = 0.0178f; // wheel radius in meters
    const float b_wheel = 0.13f;          // wheelbase, distance from wheel to wheel in meters
    // const float r1_wheel = 0.0178f; // right wheel radius in meters
    // const float r2_wheel = 0.01785f; // left  wheel radius in meters
    // const float b_wheel  = 0.158f; // wheelbase, distance from wheel to wheel in meters
    // const float radius_ratio = r1_wheel / r2_wheel;
    // line follower, tune max. vel rps to your needs
    LineFollower lineFollower(PB_9, PB_8, bar_dist, (r_wheel*2), b_wheel, motor_M2.getMaxPhysicalVelocity());

    const float Kp = 1.0f * 2.0f;
    const float Kp_nl = 1.0f * 17.0f;
    float wheel_vel_max = motor_M2.getMaxPhysicalVelocity();
    lineFollower.setRotationalVelocityControllerGains(Kp, Kp_nl);
    lineFollower.setMaxWheelVelocity(wheel_vel_max);


    // transforms wheel to robot velocities
    Eigen::Matrix2f Cwheel2robot;
    // Cwheel2robot <<  r1_wheel / 2.0f   ,  r2_wheel / 2.0f   ,
    //                 r1_wheel / b_wheel, -r2_wheel / b_wheel;
    Cwheel2robot <<  r_wheel / 2.0f   ,  r_wheel / 2.0f   ,
                r_wheel / b_wheel, -r_wheel / b_wheel;

    // rotational velocity controller
    // const float Kp{10.0f};
    // const float wheel_vel_max = 2.0f * M_PIf * motor_M2.getMaxPhysicalVelocity();

    // const float L_square = 1.4f;      // forward distance in meters
    // const float turn = -M_PIf / 2.0f; // rotation angle in radians
    //                                   // -90 deg for right turn (CW), +90 deg for left turn (CCW)

    

    // enable hardwaredriver DC motors: 0 -> disabled, 1 -> enabled
    enable_motors = 1;

    // Eigen::Vector2f robot_coord = {0.0f, 0.0f};  // contains v and w (robot translational and rotational velocity)
    // Eigen::Vector2f wheel_speed = {0.0f, 0.0f};  // contains w1 and w2 (wheel speed)
    // calculate pure forward and pure turn movement as wheel angles
    // Eigen::Vector2f robot_coord_forward = {L_square, 0.0f};
    // Eigen::Vector2f robot_coord_turn = {0.0, turn};
    // Eigen::Vector2f wheel_angle_forward = Cwheel2robot.inverse() * robot_coord_forward;
    // Eigen::Vector2f wheel_angle_turn = Cwheel2robot.inverse() * robot_coord_turn;

    // // set robot velocities
    // robot_coord(0) = 1.0f; // set desired translational velocity in m/s
    // robot_coord(1) = 0.5f; // set desired rotational velocity in rad/s


    // // map robot velocities to wheel velocities in rad/sec
    // wheel_speed = Cwheel2robot.inverse() * robot_coord;

    // setpoints for the dc motors in rps
    // motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M1
    // motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M2

    // // set up states for state machine
    // enum RobotState {
    //     FORWARD,
    //     TURN,
    //     FORWARD_OR_RESET,
    //     RESET
    // } robot_state = RobotState::FORWARD;

    // const float angle_threshold = 0.005f;
    // int turn_cntr = 0;


    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        if (do_execute_main_task) {

            // // only update sensor bar angle if an led is triggered
            // if (sensor_bar.isAnyLedActive())
            //     angle = sensor_bar.getAvgAngleRad();

            // // print to the serial terminal
            // printf("Averaged Bar Raw: |  %0.2f  | %0.2f |  %0.2f |  %0.2f |  %0.2f |  %0.2f |  %0.2f |  %0.2f | ", sensor_bar.getAvgBit(0)
            //                                                                                 , sensor_bar.getAvgBit(1)
            //                                                                                 , sensor_bar.getAvgBit(2)
            //                                                                                 , sensor_bar.getAvgBit(3)
            //                                                                                 , sensor_bar.getAvgBit(4)
            //                                                                                 , sensor_bar.getAvgBit(5)
            //                                                                                 , sensor_bar.getAvgBit(6)
            //                                                                                 , sensor_bar.getAvgBit(7));
            // printf("Mean Left: %0.2f, Mean Center: %0.2f, Mean Right: %0.2f \n", sensor_bar.getMeanThreeAvgBitsLeft()
            //                                             , sensor_bar.getMeanFourAvgBitsCenter()
            //                                             , sensor_bar.getMeanThreeAvgBitsRight());

            // // control algorithm for robot velocities
            // Eigen::Vector2f robot_coord = {0.5f * wheel_vel_max * r_wheel,  // half of the max. forward velocity
            //                             Kp * angle                    }; // simple proportional angle controller

            // // map robot velocities to wheel velocities in rad/sec
            // Eigen::Vector2f wheel_speed = Cwheel2robot.inverse() * robot_coord;

            // // setpoints for the dc motors in rps
            // motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M1
            // motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M2
            // setpoints for the dc motors in rps
            motor_M1.setVelocity(lineFollower.getRightWheelVelocity()); // set a desired speed for speed controlled dc motors M1
            motor_M2.setVelocity(lineFollower.getLeftWheelVelocity());  // set a desired speed for speed controlled dc motors M2

        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // reset variables and objects
                // led1 = 0;
                enable_motors = 0;
            }
        }


        // toggling the user led
        // user_led = !user_led;

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

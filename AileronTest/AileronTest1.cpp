#include "../libraries/library_linker.h"
#include "../libraries/modbus_client/device_applications/actuator.h"
#include <iostream>
#include <winuser.h>
#include <conio.h>
#include <thread>
using namespace std;

#define KEY_UP      72
#define KEY_DOWN    80
#define KEY_ESCAPE  27

#define NUM_MOTORS 2
Actuator motors[NUM_MOTORS]{
  {0, "Orca A", 1}
, {0, "Orca B", 1}
};
Actuator::ConnectionConfig connection_params;

uint16_t spring_configuration[6] = { 6000                //spring gain
                                 , (uint16_t)(65000)    //spring center low register
                                 , (65000 >> 16)        //spring center high register
                                 ,  0                   //spring coupling 
                                 ,  0                   //spring dead zone 
                                 ,  0 };                 //spring saturation

int32_t max_positions[2] = { {130000}, {130000} };      //this is the total range of positions used to normalize position, this can be used to allow small movements on one motor to convert to large movements on another

bool mode_set = false;
int32_t final_target[2];        //Spring center target positions

float aileron_defaults[2] = { max_positions[0] / 2, max_positions[1] / 2 };
float aileron_targets[2] = { aileron_defaults[0], aileron_defaults[1] };
float aileron_up_vals[2] = { max_positions[0] * 9/16, max_positions[1] * 9/16 };
float aileron_down_vals[2] = { max_positions[0] * 7/16, max_positions[1] * 7/16 };
int aileron_force;

int port_number[NUM_MOTORS];

/** @brief This function does the basic set up to set the spring configuration and enable effects, then calculates the appropriate spring center for each motor.
*/
void calculate_targets_haptic() {

    if (!mode_set) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i].write_registers(S0_GAIN_N_MM, 6, spring_configuration);
            motors[i].write_register(HAPTIC_STATUS, Actuator::Spring0);
            motors[i].set_mode(Actuator::HapticMode);
        }
        mode_set = true;
    }

    final_target[0] = aileron_targets[0];
    final_target[1] = aileron_targets[1];

    motors[0].update_write_stream(2, S0_CENTER_UM, final_target[0]);
    motors[1].update_write_stream(2, S0_CENTER_UM, final_target[1]);
}


//timer is used to allow smooth communications.
void motor_comms() {
    while (1) {
        if (motors[0].is_connected() && motors[1].is_connected()) {
            calculate_targets_haptic();
        }

        for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i].run_in();
            motors[i].run_out();
        }
    }
}

int main()
{
    //allow user to choose comports to connect motors on
    cout << "Aileron Demo Connect 2 motors to begin. Ensure Comport Latency set to 1 ms in device manager" << endl;
    cout << endl << "Enter port of the motor A's RS422" << endl;
    while (1) {
        string port_input;
        getline(cin, port_input);
        try {
            port_number[0] = stoi(port_input);
            break;
        }
        catch (exception e) {
            cout << "Error with entry. Please enter an integer." << endl;
        }
    }
    cout << "Enter port of the motor B's RS422" << endl;
    while (1) {
        string port_input;
        getline(cin, port_input);
        try {
            port_number[1] = stoi(port_input);
            break;
        }
        catch (exception e) {
            cout << "Error with entry. Please enter an integer." << endl;
        }
    }

    cout << "Using ports " + String(port_number[0]) + " and " + String(port_number[1]) << endl;

    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].set_new_comport(port_number[i]);
        connection_params.target_baud_rate_bps = 1250000;
        connection_params.target_delay_us = 0;
        motors[i].set_connection_config(connection_params);
        motors[i].init();
        motors[i].set_stream_mode(Actuator::MotorWrite);
        motors[i].enable();
    }

    thread mthread(motor_comms); //process motor communications in seperate thread

    cout << endl << "W: Aileron Up" << endl;
    cout << "A: Aileron Left" << endl;
    cout << "S: Aileron Down" << endl;
    cout << "D: Aileron Right" << endl;
    cout << endl << "Up Arrow: Resume Aileron Control" << endl;
    cout << "Down Arrow: Pause Aileron Control" << endl;
    cout << "Escape: Close Program" << endl;

    int c;
    while (1) {
        if (GetKeyState('W') < 0) {
            aileron_targets[0] = aileron_up_vals[0];
            aileron_targets[1] = aileron_up_vals[1];
        } else if (GetKeyState('A') < 0) {
            aileron_targets[0] = aileron_down_vals[0];
            aileron_targets[1] = aileron_up_vals[1];
        } else if (GetKeyState('S') < 0) {
            aileron_targets[0] = aileron_down_vals[0];
            aileron_targets[1] = aileron_down_vals[1];
        } else if (GetKeyState('D') < 0) {
            aileron_targets[0] = aileron_down_vals[0];
            aileron_targets[1] = aileron_up_vals[1];
        } else {
            aileron_targets[0] = aileron_defaults[0];
            aileron_targets[1] = aileron_defaults[1];
            c = 0;
            switch ((c = _getch())) {
                case KEY_UP:
                    motors[0].set_mode(Actuator::HapticMode);
                    motors[1].set_mode(Actuator::HapticMode);
                    cout << "Resuming Aileron Control" << endl;
                    break;
                case KEY_DOWN:
                    motors[0].set_mode(Actuator::SleepMode);
                    motors[1].set_mode(Actuator::SleepMode);
                    cout << "Motors to Sleep" << endl;
                    cout << "Press Up Arrow to return Control" << endl;
                    break;
                case KEY_ESCAPE:
                    motors[0].set_mode(Actuator::SleepMode);
                    motors[1].set_mode(Actuator::SleepMode);
                    exit(0);
                    break;
            }
        }
    }
    return 1;
}

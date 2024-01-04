#include <library_linker.h>
#include <actuator.h>
#include <iostream>
#include <stdlib.h>
#include <thread>
#define SDL_MAIN_HANDLED
#include <SDL.h>
using namespace std;

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
float aileron_up_vals[2] = { max_positions[0] * 9 / 16, max_positions[1] * 9 / 16 };
float aileron_down_vals[2] = { max_positions[0] * 7 / 16, max_positions[1] * 7 / 16 };
float aileron_joystick_weight[2][2] = { { max_positions[0] * 7 / 16, max_positions[0] * 2 / 16 },
    { max_positions[1] * 7 / 16, max_positions[1] * 2 / 16 } };
int aileron_force;

int port_number[NUM_MOTORS];

/** @brief This function does the basic set up to set the spring configuration and enable effects, then calculates the appropriate spring center for each motor.
*/
void calculate_targets_haptic() {

    if (!mode_set) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i].write_registers(S0_GAIN_N_MM, 6, spring_configuration);
            motors[i].write_register(I0_GAIN_NS2_MM, uint16_t(0.001));
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

void startText() {
    cout << "------------------------------------------------------------------" << endl;
    cout << "Welcome to the Merlin Machine Works joystick test using IRIS Orcas" << endl;
    cout << "------------------------------------------------------------------" << endl << endl;
}

void instructText() {
    cout << endl << "-Y: Aileron Up" << endl;
    cout << " X: Aileron Left" << endl;
    cout << "-X: Aileron Down" << endl;
    cout << " Y: Aileron Right" << endl;
    cout << endl << "Button 5: Resume Aileron Control" << endl;
    cout << "Button 6: Pause Aileron Control" << endl;
    cout << "Button 7: Close Program" << endl << endl;
}

int main()
{
    startText();

    //SDL2 stuff for joystick control
    SDL_Init(SDL_INIT_JOYSTICK);
    SDL_JoystickEventState(SDL_ENABLE);
    if (SDL_NumJoysticks() == 0) {
        cout << "Your joystick is not connected! Please plug in a joystick" << endl;
        while (SDL_NumJoysticks() == 0) {
            SDL_Init(SDL_INIT_JOYSTICK);
        }
        system("cls");
        startText();
    }
    SDL_Joystick* joy;
    joy = SDL_JoystickOpen(0);

    //allow user to choose comports to connect motors on
    cout << "Connect 2 motors to begin. Ensure Comport Latency set to 1 ms in device manager" << endl;
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

    system("cls");
    startText();

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

    instructText();

    while (1) {
        SDL_Event ev;

        while (SDL_PollEvent(&ev))
        {
            if(SDL_JoystickGetButton(joy, 4)) {
                motors[0].set_mode(Actuator::HapticMode);
                motors[1].set_mode(Actuator::HapticMode);
                cout << "Resuming Aileron Control" << endl;
            }
            else if (SDL_JoystickGetButton(joy, 5)) {
                motors[0].set_mode(Actuator::SleepMode);
                motors[1].set_mode(Actuator::SleepMode);
                cout << "Motors to Sleep" << endl;
                cout << "Press Up Arrow to return Control" << endl;
            }
            else if (SDL_JoystickGetButton(joy, 6)) {
                motors[0].set_mode(Actuator::SleepMode);
                motors[1].set_mode(Actuator::SleepMode);
                exit(1);
            }

            /*float joyCalc[2]{ aileron_defaults[0] + (SDL_JoystickGetAxis(joy, 1) / -32767.0 * aileron_joystick_weight[0][1] + SDL_JoystickGetAxis(joy, 0) / 32767.0 * aileron_joystick_weight[0][0]),
                aileron_defaults[1] + (SDL_JoystickGetAxis(joy, 1) / -32767.0 * aileron_joystick_weight[1][1] - SDL_JoystickGetAxis(joy, 0) / 32767.0 * aileron_joystick_weight[1][0])
            };*/

            float joyCalc[2]{ aileron_defaults[0] + SDL_JoystickGetAxis(joy, 0) / 32767.0 * aileron_joystick_weight[0][0],
                aileron_defaults[1] - SDL_JoystickGetAxis(joy, 0) / 32767.0 * aileron_joystick_weight[1][0]
            };

            aileron_targets[0] = joyCalc[0];
            aileron_targets[1] = joyCalc[1];
        }
    }

    return 0;
}
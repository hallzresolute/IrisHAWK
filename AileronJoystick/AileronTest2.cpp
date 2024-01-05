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

#define Y_AXIS_USED false

/** @brief This function does the basic set up to set the spring configuration and enable effects, then calculates the appropriate spring center for each motor.
*/
void calculate_targets_haptic() {

    if (!mode_set) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i].write_register(D0_GAIN_NS_MM, uint16_t(5000));
            motors[i].write_register(HAPTIC_STATUS, Actuator::Damper);
            motors[i].write_register(I0_GAIN_NS2_MM, uint16_t(5000));
            motors[i].write_register(HAPTIC_STATUS, Actuator::Inertia);
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

void startText() {
    system("cls");
    cout << "  /------------------------------------------------------------------\\ " << endl;
    cout << " /                                                                    \\ " << endl;
    cout << "|  Welcome to the Merlin Machine Works joystick test using IRIS Orcas  |" << endl;
    cout << " \\                                                                    /" << endl;
    cout << "  \\------------------------------------------------------------------/" << endl << endl;
}

void instructText() {
    if (Y_AXIS_USED) {
        cout << " -Y Axis: Aileron Up" << endl;
        cout << "  Y Axis: Aileron Down" << endl;
    }
    cout << "  X Axis: Aileron Left" << endl;
    cout << " -X Axis: Aileron Right" << endl << endl;
    
    cout << "Button 5: Resume Motor Control" << endl;
    cout << "Button 6: Pause Motor Control" << endl;
    cout << "Button 7: Close Program" << endl << endl;
    cout << "-----------------------------------------------------------------------" << endl << endl;
}

void portText() {
    cout << "Using ports " + String(port_number[0]) + " and " + String(port_number[1]) << endl << endl;
}

void allRunningText() {
    startText();
    portText();
    instructText();
}

int main()
{
    startText();

    //SDL2 stuff for joystick control
    SDL_Init(SDL_INIT_JOYSTICK);
    SDL_JoystickEventState(SDL_ENABLE);
    if (SDL_NumJoysticks() == 0) {
        cout << "No joystick found, please plug in a joystick" << endl;
        while (SDL_NumJoysticks() == 0) {
            SDL_Init(SDL_INIT_JOYSTICK);
        }
        startText();
    }
    SDL_Joystick* joy;
    joy = SDL_JoystickOpen(0);

    //allow user to choose comports to connect motors on
    cout << "Connect 2 motors to begin. Ensure Comport Latency is set to 1 ms in Device Manager" << endl;
    cout << endl << "Enter port of the motor A's RS422: ";
    while (1) {
        string port_input;
        getline(cin, port_input);
        try {
            port_number[0] = stoi(port_input);
            break;
        }
        catch (exception e) {
            cout << "Error with entry, please enter an integer" << endl;
        }
    }
    cout << "Enter port of the motor B's RS422: ";
    while (1) {
        string port_input;
        getline(cin, port_input);
        try {
            port_number[1] = stoi(port_input);
            break;
        }
        catch (exception e) {
            cout << "Error with entry, please enter an integer" << endl;
        }
    }

    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].set_new_comport(port_number[i]);
        connection_params.target_baud_rate_bps = 1250000;
        connection_params.target_delay_us = 0;
        motors[i].set_connection_config(connection_params);
        motors[i].init();
        motors[i].set_stream_mode(Actuator::MotorWrite);
        motors[i].enable();
    }

    //process motor communications in seperate thread
    thread mthread(motor_comms);

    allRunningText();

    bool slep = false;

    while (1) {
        SDL_Event ev;

        while (SDL_PollEvent(&ev))
        {
            if(SDL_JoystickGetButton(joy, 4) && slep) {
                slep = false;
                motors[0].set_mode(Actuator::HapticMode);
                motors[1].set_mode(Actuator::HapticMode);
                allRunningText();
                cout << "Motor Control Resumed" << endl;
            }
            else if (SDL_JoystickGetButton(joy, 5) && !slep) {
                slep = true;
                motors[0].set_mode(Actuator::SleepMode);
                motors[1].set_mode(Actuator::SleepMode);
                allRunningText();
                cout << "Motors to Sleep" << endl;
            }
            else if (SDL_JoystickGetButton(joy, 6)) {
                motors[0].set_mode(Actuator::SleepMode);
                motors[1].set_mode(Actuator::SleepMode);
                exit(1);
            }

            float joyCalc[2];

            if (Y_AXIS_USED) {
                joyCalc[0] = aileron_defaults[0] + (SDL_JoystickGetAxis(joy, 1) / -32767.0 * aileron_joystick_weight[0][1] + SDL_JoystickGetAxis(joy, 0) / 32767.0 * aileron_joystick_weight[0][0]);
                joyCalc[1] = aileron_defaults[1] + (SDL_JoystickGetAxis(joy, 1) / -32767.0 * aileron_joystick_weight[1][1] - SDL_JoystickGetAxis(joy, 0) / 32767.0 * aileron_joystick_weight[1][0]);
            }
            else {
                joyCalc[0] = aileron_defaults[0] + SDL_JoystickGetAxis(joy, 0) / 32767.0 * aileron_joystick_weight[0][0];
                joyCalc[1] = aileron_defaults[1] - SDL_JoystickGetAxis(joy, 0) / 32767.0 * aileron_joystick_weight[1][0];
            }

            aileron_targets[0] = joyCalc[0];
            aileron_targets[1] = joyCalc[1];
        }
    }

    return 0;
}
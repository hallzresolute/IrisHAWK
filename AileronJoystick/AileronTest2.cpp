#include <library_linker.h>
#include <actuator.h>
#include <iostream>
#include <stdlib.h>
#include <thread>
using namespace std;

#define SDL_MAIN_HANDLED
#include <SDL.h>

#define NUM_MOTORS 2
Actuator motors[NUM_MOTORS]{
  {0, "Orca A", 1}
, {0, "Orca B", 1}
};
Actuator::ConnectionConfig connection_params;

uint16_t damper_configuration = 5000;           //desired motor dampening
uint16_t inertia_configuration = 5000;          //desired motor inertial weight

uint16_t spring_configuration[6] = { 6000               //spring gain
                                 , (uint16_t)(65000)    //spring center low register
                                 , (65000 >> 16)        //spring center high register
                                 ,  0                   //spring coupling 
                                 ,  0                   //spring dead zone 
                                 ,  0 };                //spring saturation

int max_pos[2] = { {130000}, {130000} };            //the total range of motor positions

int pos_defs[2] = { max_pos[0] / 2, max_pos[1] / 2 };         //our "back to center" values
int spring_targets[2] = { pos_defs[0], pos_defs[1] };       //spring center target positions

int joy_weights[2][2] = {                        //How far the motor moves at max joystick throw. First dimension is the motor, second dimension is the axis(X, Y)
    { max_pos[0] * 7 / 16, max_pos[0] * 2 / 16 },
    { max_pos[1] * 7 / 16, max_pos[1] * 2 / 16 }
};

int port_numbers[NUM_MOTORS];        //motor RS422 ports

/*  Joseph only wants aileron control, but I originally had functionality for up and down movement as well.
    I'm leaving this here in case we want it down the line*/
#define Y_AXIS_USED false

bool mode_set = false;      //determines if the motor has been inited into its haptic mode

//initializes the motors with the haptics we want, and does the actual motor movement
void calculate_targets_haptic() {
    if (!mode_set) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i].write_register(D0_GAIN_NS_MM, damper_configuration);
            motors[i].write_register(HAPTIC_STATUS, Actuator::Damper);
            motors[i].write_register(I0_GAIN_NS2_MM, inertia_configuration);
            motors[i].write_register(HAPTIC_STATUS, Actuator::Inertia);
            motors[i].write_registers(S0_GAIN_N_MM, 6, spring_configuration);
            motors[i].write_register(HAPTIC_STATUS, Actuator::Spring0);
            motors[i].set_mode(Actuator::HapticMode);
        }
        mode_set = true;
    }

    motors[0].update_write_stream(2, S0_CENTER_UM, spring_targets[0]);
    motors[1].update_write_stream(2, S0_CENTER_UM, spring_targets[1]);
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

//just for a pretty CLI
void startText() {
    system("cls");
    cout << "  /------------------------------------------------------------------\\ " << endl;
    cout << " /                                                                    \\ " << endl;
    cout << "|  Welcome to the Merlin Machine Works joystick test using IRIS Orcas  |" << endl;
    cout << " \\                                                                    /" << endl;
    cout << "  \\------------------------------------------------------------------/" << endl << endl;
}

//just for a pretty CLI
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

//just for a pretty CLI
void portText() {
    cout << "Using ports " + String(port_numbers[0]) + " and " + String(port_numbers[1]) << endl << endl;
}

//just for a pretty CLI
void allRunningText() {
    startText();
    portText();
    instructText();
}


/////////////////////////
//                     //
//      MAIN FUNC      //
//                     //
/////////////////////////
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
            port_numbers[0] = stoi(port_input);
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
            port_numbers[1] = stoi(port_input);
            break;
        }
        catch (exception e) {
            cout << "Error with entry, please enter an integer" << endl;
        }
    }

    //goofy ah 1250000 baud rate
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].set_new_comport(port_numbers[i]);
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

    //only allows event loop to do sleeps and resumes when it needs to
    bool mot_asleep = false;

    /////////////////////////////
    //                         //
    //      RUNNING LOOP       //
    //                         //
    /////////////////////////////
    while (1) {
        SDL_Event ev;

        //event loop when joystick is doing things
        while (SDL_PollEvent(&ev))
        {
            if(SDL_JoystickGetButton(joy, 4) && mot_asleep) {
                mot_asleep = false;
                motors[0].set_mode(Actuator::HapticMode);
                motors[1].set_mode(Actuator::HapticMode);
                allRunningText();
                cout << "Motor Control Resumed" << endl;
            }
            else if (SDL_JoystickGetButton(joy, 5) && !mot_asleep) {
                mot_asleep = true;
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

            //calculate desired spring center based on joystick positions
            if (Y_AXIS_USED) {
                spring_targets[0] = pos_defs[0] + (SDL_JoystickGetAxis(joy, 1) / -32767.0 * joy_weights[0][1] + SDL_JoystickGetAxis(joy, 0) / 32767.0 * joy_weights[0][0]);
                spring_targets[1] = pos_defs[1] + (SDL_JoystickGetAxis(joy, 1) / -32767.0 * joy_weights[1][1] - SDL_JoystickGetAxis(joy, 0) / 32767.0 * joy_weights[1][0]);
            }
            else {
                spring_targets[0] = pos_defs[0] + SDL_JoystickGetAxis(joy, 0) / 32767.0 * joy_weights[0][0];
                spring_targets[1] = pos_defs[1] - SDL_JoystickGetAxis(joy, 0) / 32767.0 * joy_weights[1][0];
            }
        }
    }

    return 0;
}

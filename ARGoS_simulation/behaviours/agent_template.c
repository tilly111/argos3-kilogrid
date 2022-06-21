/*-----------------------------------------------------------------------------------------------*/
/* This file gives a template of what functions need to be implemented for Kilobot in order to   */
/* work on the Kilogrid.                                                                         */
/*-----------------------------------------------------------------------------------------------*/

// macro if we are in sim or reality -> command out if on real robot
#define SIMULATION


/*-----------------------------------------------------------------------------------------------*/
/* Imports - depending on the platform one has different imports                                 */
/*-----------------------------------------------------------------------------------------------*/

#include <stdlib.h>
#include "kilolib.h"
#include <math.h>

#ifdef SIMULATION

#include <stdio.h>
#include <float.h>
#include "agent.h"
#include <debug.h>

#else

#include "utils.h"
#include "kilob_tracking.h"
#include "kilo_rand_lib.h"
#include "../communication.h"
#include "kilob_messaging.h"

#endif


/*-----------------------------------------------------------------------------------------------*/
/* Define section here you can define values, e.g., messages types                               */
/*-----------------------------------------------------------------------------------------------*/
#define PI 3.14159265358979323846
// options
#define UNCOMMITTED 0
#define UNINITIALISED 20
// message types



/*-----------------------------------------------------------------------------------------------*/
/* Enum section - here we can define useful enums                                                */
/*-----------------------------------------------------------------------------------------------*/
typedef enum{
    false = 0,
    true = 1,
} bool;


bool new_robot_msg = false;

/*-----------------------------------------------------------------------------------------------*/
/* Communication variables - used for communication and stuff                                    */
/*-----------------------------------------------------------------------------------------------*/
// how often we try to send the msg - in simulation once is sufficient
#ifdef SIMULATION
#define MSG_SEND_TRIES 1
#else
#define MSG_SEND_TRIES 10
#endif
// Kilobot -> Kilogrid
uint32_t msg_counter_sent = MSG_SEND_TRIES + 1;  // counts the messages sent
uint32_t msg_number_send = 0;  // change if you want to send a msg
uint32_t msg_number_current_send = 0;  // var for checking against the last
// Kilogrid -> Kilobot
bool init_flag = false;
bool received_grid_msg_flag = false;
bool received_virtual_agent_msg_flag = false;



/*-----------------------------------------------------------------------------------------------*/
/* Arena variables                                                                               */
/*-----------------------------------------------------------------------------------------------*/
uint8_t NUMBER_OF_OPTIONS = 0;
uint8_t current_ground = 0;
const uint8_t CELLS_X = 20;
const uint8_t CELLS_Y = 40;




/*-----------------------------------------------------------------------------------------------*/
/* Function to process the data received from the kilogrid regarding the environment             */
/*-----------------------------------------------------------------------------------------------*/
void update_grid_msg() {

}


/*-----------------------------------------------------------------------------------------------*/
/* Function to process the data received from the kilogrid regarding other robots                */
/*-----------------------------------------------------------------------------------------------*/
void update_virtual_agent_msg() {

}


/*-----------------------------------------------------------------------------------------------*/
/* This function implements the callback, for when the robot receives an infrared message (here  */
/* only from the kilogrid)                                                                       */
/*-----------------------------------------------------------------------------------------------*/
// because there has been an "updated" version of the kilo_lib we have a slightly different
// implementation
#ifdef SIMULATION
void message_rx( message_t *msg, distance_measurement_t *d ) {
#else
void message_rx( IR_message_t *msg, distance_measurement_t *d ) {
#endif
    // TODO: this is how you could implement the infrared callback
    // check the messages
    // all data should be stored in temporary variables and then be written in the loop
    // in order to dont fuck up your calculations, because this function works like an interrupt!!
//    if(msg->type == INIT_MSG && !init_flag){
//        // TODO: implement action...
//    }else if(msg->type == GRID_MSG && init_flag){
//        // TODO: implement action...
//    }else if(msg->type == VIRTUAL_AGENT_MSG  && init_flag){
//        // TODO: implement action....
//    }
}


/*-----------------------------------------------------------------------------------------------*/
/* Callback function for successful transmission                                                 */
/*-----------------------------------------------------------------------------------------------*/
void tx_message_success() {
    msg_counter_sent += 1;
    return;
}


/*-----------------------------------------------------------------------------------------------*/
/* This function implements the sending to the kilogrid. you should call this function every     */
/* loop cycle because in reality you dont have a indicator if the message was received so we     */
/* have to send it multiple times. The when and how often to send a message should be            */
/* implemented here!                                                                             */
/*-----------------------------------------------------------------------------------------------*/
void message_tx(){
    // implementation differs because in simulation we use the debugstruct - faster and easier to
    // understand
    // in reality we send infrared msg - we send more than one to make sure that the messages arrive!
    if (msg_number_current_send != msg_number_send){
        msg_number_current_send = msg_number_send;
        msg_counter_sent = 0;
    }
#ifdef SIMULATION
    /// this is needed because in simulation we use the debug struct, thus we do not really send
    /// a message
    if(msg_counter_sent >= MSG_SEND_TRIES){
        debug_info_set(broadcast_flag, 0);
    } else {
        tx_message_success();
    }
#else
    /// sending a real message - thus tx_message_success gets called anyway
    // TODO check if this is true
    if (msg_counter_sent <= MSG_SEND_TRIES){
        if((message = kilob_message_send()) != NULL) {
            /*
            message->type = TO_KILOGRID_MSG;
            message->data[0] = my_commitment;
            message->data[1] = communication_range;
            message->data[2] = robot_gps_x;
            message->data[3] = robot_gps_y;
            message->data[4] = msg_number_current_send;
            */
            msg_counter_sent += 1;

        }
    }
#endif
}





/*-----------------------------------------------------------------------------------------------*/
/* Init function                                                                                 */
/*-----------------------------------------------------------------------------------------------*/
void setup(){
#ifndef SIMULATION
    // for tracking the robot in real life
    kilob_tracking_init();
    kilob_messaging_init();
    tracking_data.byte[0] = kilo_uid;
    tracking_data.byte[5] = 0;
#endif
    // Initialise random seed
    uint8_t seed = rand_hard();
    rand_seed(seed);
    seed = rand_hard();
    srand(seed);

    // Initialise motors
    set_motors(0,0);

    // TODO initalise robots

    // Initialise time to 0
    kilo_ticks = 0;
}


/*-----------------------------------------------------------------------------------------------*/
/* Main loop                                                                                     */
/*-----------------------------------------------------------------------------------------------*/
void loop() {
    // TODO: I find it nice to initialise the robots before they start but you dont have to do it
    //  like this
    if(init_flag){  // initalization happend
        // process received msgs
        if (received_grid_msg_flag) {
            update_grid_msg();
            received_grid_msg_flag = false;
        }

        if (received_virtual_agent_msg_flag) {
            update_virtual_agent_msg();
            received_virtual_agent_msg_flag = false;
        }

        // HOW TO SEND AN INFRARED MESSAGE
        // you have to set the message: means set contents of the message and then activating the
        // broadcast flag
//        msg_number_send += 1;
//        debug_info_set(broadcast_flag, 1);
//        debug_info_set(type, MSG_T_VIRTUAL_ROBOT_MSG);
//        debug_info_set(data0, PAYLOAD);


        // for sending messages
        message_tx();
    }else{
        // not initialized yet ... can be omitted just for better understanding
        // also you can do some debugging here
    }

    // DEBUG SECTION - FOR SIMULATION NEEDED SO DO NOT DELETE
#ifdef SIMULATION
    // HOW TO SET DEBUG INFORMATION. Attention: add also to debug struct in agent.h
//    debug_info_set(commitement, robot_commitment);
#endif
}


/*-----------------------------------------------------------------------------------------------*/
/* Main function - obviously needs to be implemented by both platforms.                          */
/*-----------------------------------------------------------------------------------------------*/
int main(){
    // initialize the hardware of the robot
    kilo_init();
    // now initialize specific things only needed for one platform
#ifdef SIMULATION
    // create debug struct - mimics the communication with the kilogrid
    debug_info_create();
#else
    // initalize utils
    utils_init();
#endif
    // callback for received messages
    kilo_message_rx = message_rx;
    kilo_message_tx_success = tx_message_success;
    // start control loop
    kilo_start(setup, loop);
    return 0;
}
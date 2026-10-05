/*
    Module to handle the actuation of the Braccio arm
    Header file
    Author: Neo Vorsatz
    Last updated: 5 October 2026
*/

/*
    You have been blessed by the alicorn.
    May your code compile without errors nor warnings, and may your memory access be forever safe.

                                                       Bradley

                  <<<<>>>>>>           .----------------------------.
               _>><<<<>>>>>>>>>       /               _____________)
      \|/      \<<<<<  < >>>>>>>>>   /            _______________)
-------*--===<=<<           <<<<<<<>/         _______________)
      /|\     << @    _/      <<<<</       _____________)
             <  \    /  \      >>>/      ________)  ____
                 |  |   |       </      ______)____((- \\\\
                 o_|   /        /      ______)         \  \\\\    \\\\\\\
                      |  ._    (      ______)           \  \\\\\\\\\\\\\\\\
                      | /       `----------'    /       /     \\\\\\\     \\
              .______/\/     /                 /       /          \\\
             / __.____/    _/         ________(       /\
            / / / ________/`---------'         \     /  \_
           / /  \ \                             \   \ \_  \
          ( <    \ \                             >  /    \ \
           \/      \\_                          / /       > )
                    \_|                        / /       / /
                                             _//       _//
                                            /_|       /_|

*/

#ifndef BRACCIO_ACTUATION_H
#define BRACCIO_ACTUATION_H

// External includes
#include <Arduino.h>
#include <Servo.h>

// DATA STRUCTURES ================================

typedef struct {
    float base;
    float joint1;
    float joint2;
    float joint3;
    float wrist;
    float claw;
} vector_t;

// CONSTANTS ================================

// Soft start
#define BA_DEFAULT_SOFT_START 20 // approximately 1 second start-up
#define BA_DISABLE_SOFT_START 0
#define BA_MAX_SOFT_START 40 // approximately 0.5 second start-up
#define BA_MIN_SOFT_START 5 // approximately 4 seconds start-up

// Connectors
#define BA_M1 0
#define BA_M2 1
#define BA_M3 2
#define BA_M4 3
#define BA_M5 4
#define BA_M6 5

// Default connector arrangement
const int BA_DEFAULT_CONN[6] = {BA_M1, BA_M2, BA_M3, BA_M4, BA_M5, BA_M6};

// Positional error tolerance
#define BA_POS_TOLERANCE 25.0f // in squared degrees

// INITIALIZERS AND CREATION ================================

/**
 * @brief Initializes the Braccio arm operation
 * 
 * @param softStart The soft start value
 * @param conn The connector arrangement/order. Use `BA_M1`, `BA_M2`, ..., `BA_M6` to reconfigure
 * 
 * @return `true` if the initialization was successful, `false` otherwise
 * 
 * @note The soft start value increases the rate of the soft start (makes it less soft).
 * @note Giving BA_DISABLE_SOFT_START frees pin 12, for Braccio shields older than V4.
 * @note Disabling soft start is recommended for the Braccio shield V1.6
 */
bool baInit(int softStart=BA_DEFAULT_SOFT_START, const int conn[6]=BA_DEFAULT_CONN);

/**
 * @brief Creates a new vector
 * 
 * @param base The value for the base
 * @param joint1 The value for the first joint
 * @param joint2 The value for the second joint
 * @param joint3 The value for the third joint
 * @param wrist The value for the wrist
 * @param claw The value for the claw
 * 
 * @return The new vector
 */
vector_t baNewVector(float base, float joint1, float joint2, float joint3, float wrist, float claw);

// READING ================================

/**
 * @brief Gets the desired position the Braccio arm is aiming for
 * 
 * @return The desired position
 */
vector_t baGetTarget();

/**
 * @brief Gets the current position of the Braccio arm
 * 
 * @return The current position
 */
vector_t baGetPosition();

/**
 * @brief Gets the error between the desired position and the current position
 * 
 * @return The sum of squared position errors
 */
float baGetError();

/**
 * @brief Checks if the Braccio arm is at the desired position
 * 
 * @param tolerance The squared positional tolerance
 * 
 * @return `true` if the arm is at the desired position, otherwise `false`
 */
bool baAtTarget(float tolerance=BA_POS_TOLERANCE);

// CONTROLLING ================================

/**
 * @brief Sets the position of the Braccio arm
 * 
 * @param position The position to set the arm to
 * 
 * @return `true` if the position is within the mechanical constraints, otherwise `false`
 * 
 * @note This function is not recommended, because it'll cause jerking
 */
bool baSetPosition(vector_t position);

/**
 * @brief Updates the desired position of the Braccio arm
 *
 * @param target The desired position
 * 
 * @return `true` if the position is within the mechanical constraints, otherwise `false`
 */
bool baUpdateTarget(vector_t target);

/**
 * @brief Steps the Braccio arm towards the desired position
 * 
 * @note This function is recommended, and should be called within a loop, along with `baUpdateTarget`.
 * @note This function is computationally expensive.
 */
void baStep();

/**
 * @brief Handles the movement of the Braccio arm completely
 * 
 * @param target The desired position
 * 
 * @return `true` if the position is within the mechanical constraints, otherwise `false`
 *
 * @note This function is blocking; occupies a thread until the desired position is reached.
 * @note The function stops "simulating" at the end, even if there's remaining "velocity",
 * @note resulting in sudden stops.
 */
bool baMove(vector_t target);

/**
 * @brief Sets the time constant for 1 stage of the cascaded 2nd order low-pass filter
 * 
 * @param tau The new time constant
 */
void baSetFilterTau(float tau);

/**
 * @brief Sets the cutoff frequency (Hz) for 1 stage of the cascaded 2nd order low-pass filter
 * 
 * @param freq The new cutoff frequency
 * 
 * @note The filter's characteristic is stored as a time constant (tau), making `baSetFilterTau()` more efficient.
 * @note You can manually calculate the time constant as: tau=1/(2*pi*freq)
 */
void baSetFilterFreq(float freq);

// MANAGEMENT ================================

/**
 * @brief Checks if the target is reachable, given mechanical constraints
 *
 * @param target The desired position
 * 
 * @return `true` if the target is reachable, otherwise `false`
 */
bool baTargetReachable(vector_t target);

/**
 * @brief Bounds a pos by the mechanical constraints
 * 
 * @param position The position to bound
 * 
 * @return `true` if the position was within the mechanical constraints, otherwise `false`
 * 
 * @note Most controlling functions call this automatically
 */
bool baBoundPosition(vector_t *position);

#endif

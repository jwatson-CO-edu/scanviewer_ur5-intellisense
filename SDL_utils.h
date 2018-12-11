#pragma once // This also helps things not to be loaded twice , but not always . See below

/***********  
SDL_utils.h
James Watson , 2018 October
Convenience functions for SDL2

Template Version: 2018-07-16
***********/

#ifndef SDL_UTILS_H // This pattern is to prevent symbols to be loaded multiple times
#define SDL_UTILS_H // from multiple imports

#define SDL_FLOAT // Use floats for all graphics operations
//~ #define SDL_DUBBL // Use doubles for all graphics operations

// ~~ Includes ~~
// ~ SDL2 ~
#include <SDL2/SDL.h>
#include <SDL2/SDL_mixer.h>
// ~ OpenGL ~
#ifdef __APPLE__ // This constant is always defined on Apple machines
      #include <GLUT/glut.h> // GLUT is in a different place on Apple machines
#else
      #include <GL/glut.h>
#endif
// ~ Local ~
#include <Cpp_Helpers.h> // Favorite C++ tricks! I am the author , Source: https://bitbucket.org/jwatson_utah_edu/cpp_helpers/src/master/

// ~~ Shortcuts and Aliases ~~
// ~~ Shortcuts and Aliases ~~
#ifdef SDL_FLOAT
    using typeF = float;
#endif
#ifdef SDL_DUBBL
    using typeF = double;
#endif

// ~~ Constants ~~


// === Classes and Structs =================================================================================================================

// == class SDL_Heartbeat ==

class SDL_Heartbeat{
    // Class to maintain a maximum framerate
public:

    // ~ Con/Dstructors ~
    
    SDL_Heartbeat( typeF interval ); // Create a heartbeat with a minimum inteval
    
    // ~ Timekeeping ~
    
    void  mark_time(); // Mark the current time as the beginning of the current interval
    
    void  sleep_remainder(); // Sleep the remaining time to maintain the interval

    typeF seconds_elapsed(); // Get the number of seconds elapsed from the last mark

private:
    
    double interval_s = 1.0; // Minimum time between calls
    double lastMark_s = 0.0; // Last time that 'sleep_remainder' was called
    
};

// __ End SDL_Heartbeat __

// ___ End Classes _________________________________________________________________________________________________________________________



// === Functions ===========================================================================================================================

void PrintEvent( const SDL_Event * event );

void PrintSDL( const char* format , ... );

// ___ End Func ____________________________________________________________________________________________________________________________


#endif

/* === Spare Parts =========================================================================================================================



   ___ End Parts ___________________________________________________________________________________________________________________________

*/


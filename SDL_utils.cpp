/***********  
SDL_utils.cpp
James Watson , 2018 October
Convenience functions for SDL2

Template Version: 2018-06-07
***********/

#include "SDL_utils.h"

// === Classes and Structs =================================================================================================================

// == class SDL_Heartbeat ==

// ~ Con/Dstructors ~

SDL_Heartbeat::SDL_Heartbeat( typeF interval ){
    // Create a heartbeat with a minimum inteval
    interval_s = (double) interval;
    mark_time();
}

// ~ Timekeeping ~

// Mark the current time as the beginning of the current interval
void SDL_Heartbeat::mark_time(){  lastMark_s = (typeF)SDL_GetTicks()/1000.0;  }

void SDL_Heartbeat::sleep_remainder(){
    // Sleep the remaining time to maintain the interval
    typeF currTime = SDL_GetTicks() / 1000.0;
    typeF deltaT = currTime - lastMark_s;
    if( deltaT < interval_s ){  
        SDL_Delay( (uint) ( ( interval_s - deltaT ) * 1000.0 ) );  
    }
    mark_time();
}

typeF SDL_Heartbeat::seconds_elapsed(){
    // Get the number of seconds elapsed from the last mark
    return (typeF) SDL_GetTicks() / 1000.0 - lastMark_s;
}

// __ End SDL_Heartbeat __

// ___ End Classes _________________________________________________________________________________________________________________________



// === Functions ===========================================================================================================================

void PrintEvent(const SDL_Event * event){
    // Print information about a window event
    // Source: https://wiki.libsdl.org/SDL_WindowEvent
    switch( event->type ){
        case SDL_WINDOWEVENT:
            switch (event->window.event){
                case SDL_WINDOWEVENT_SHOWN:
                    SDL_Log( "Window %d shown" , event->window.windowID );
                    printf(  "Window %d shown" , event->window.windowID );
                    break;
                case SDL_WINDOWEVENT_HIDDEN:
                    SDL_Log( "Window %d hidden" , event->window.windowID );
                    printf(  "Window %d hidden" , event->window.windowID );
                    break;
                case SDL_WINDOWEVENT_EXPOSED:
                    SDL_Log( "Window %d exposed" , event->window.windowID );
                    printf(  "Window %d exposed" , event->window.windowID );
                    break;
                case SDL_WINDOWEVENT_MOVED:
                    SDL_Log( "Window %d moved to %d,%d" ,
                             event->window.windowID , event->window.data1 ,
                             event->window.data2 );
                    printf(  "Window %d moved to %d,%d" ,
                             event->window.windowID , event->window.data1 ,
                             event->window.data2 );
                    break;
                case SDL_WINDOWEVENT_RESIZED:
                    SDL_Log( "Window %d resized to %dx%d" ,
                             event->window.windowID , event->window.data1 ,
                             event->window.data2) ;
                    printf(  "Window %d resized to %dx%d" ,
                             event->window.windowID , event->window.data1 ,
                             event->window.data2 );
                    break;
                case SDL_WINDOWEVENT_SIZE_CHANGED:
                    SDL_Log( "Window %d size changed to %dx%d" ,
                             event->window.windowID , event->window.data1 ,
                             event->window.data2 );
                    printf(  "Window %d size changed to %dx%d" ,
                             event->window.windowID , event->window.data1 ,
                             event->window.data2 );
                    break;
                case SDL_WINDOWEVENT_MINIMIZED:
                    SDL_Log( "Window %d minimized" , event->window.windowID );
                    printf(  "Window %d minimized" , event->window.windowID );
                    break;
                case SDL_WINDOWEVENT_MAXIMIZED:
                    SDL_Log( "Window %d maximized" , event->window.windowID );
                    printf( "Window %d maximized"  , event->window.windowID );
                    break;
                case SDL_WINDOWEVENT_RESTORED:
                    SDL_Log( "Window %d restored" , event->window.windowID );
                    printf(  "Window %d restored" , event->window.windowID );
                    break;
                case SDL_WINDOWEVENT_ENTER:
                    SDL_Log( "Mouse entered window %d" ,
                             event->window.windowID    );
                    printf(  "Mouse entered window %d" ,
                             event->window.windowID    );
                    break;
                case SDL_WINDOWEVENT_LEAVE:
                    SDL_Log( "Mouse left window %d" , event->window.windowID );
                    printf(  "Mouse left window %d" , event->window.windowID );
                    break;
                case SDL_WINDOWEVENT_FOCUS_GAINED:
                    SDL_Log( "Window %d gained keyboard focus",
                             event->window.windowID );
                    printf(  "Window %d gained keyboard focus",
                             event->window.windowID );
                    break;
                case SDL_WINDOWEVENT_FOCUS_LOST:
                    SDL_Log( "Window %d lost keyboard focus",
                             event->window.windowID );
                    printf(  "Window %d lost keyboard focus",
                             event->window.windowID );
                    break;
                case SDL_WINDOWEVENT_CLOSE:
                    SDL_Log( "Window %d closed", event->window.windowID );
                    printf(  "Window %d closed", event->window.windowID );
                    break;
                #if SDL_VERSION_ATLEAST( 2 , 0 , 5 )
                case SDL_WINDOWEVENT_TAKE_FOCUS:
                    SDL_Log( "Window %d is offered a focus", event->window.windowID );
                    printf(  "Window %d is offered a focus", event->window.windowID );
                    break;
                case SDL_WINDOWEVENT_HIT_TEST:
                    SDL_Log( "Window %d has a special hit test", event->window.windowID );
                    printf(  "Window %d has a special hit test", event->window.windowID );
                    break;
                #endif
                default:
                    SDL_Log( "Window %d got unknown event %d",
                             event->window.windowID, event->window.event );
                    printf(  "Window %d got unknown event %d",
                             event->window.windowID, event->window.event );
                    break;
                    
            }
            break;
        // Source: https://wiki.libsdl.org/SDL_QuitEvent
        case SDL_QUIT:
            SDL_Log( "Program quit after %i ticks" , event->quit.timestamp );
            printf(  "Program quit after %i ticks" , event->quit.timestamp );
            break;
        // Source: https://wiki.libsdl.org/SDL_KeyboardEvent
        case SDL_KEYDOWN:
            SDL_Log( "Physical %s key acting as %s key" ,
                      SDL_GetScancodeName( event->key.keysym.scancode ) ,
                      SDL_GetKeyName( event->key.keysym.sym ) );
            printf( "Physical %s key acting as %s key" ,
                    SDL_GetScancodeName( event->key.keysym.scancode ) ,
                    SDL_GetKeyName( event->key.keysym.sym ) );
            break;
        default:
            SDL_Log( "UNKNOWN EVENT!" );
            printf(  "UNKNOWN EVENT!" );
            break;

    }
}

/*
 *  Convenience routine to output raster text
 *  Use VARARGS to make this more flexible
 *  Author: Willem A. (Vlakkies) Schreüder  
 */

//  I lifted this font from Brent Smit - not sure if it is original
static int font=0;
static GLubyte letters[][14] = {
    { 0x00, 0x00, 0x7e, 0xe7, 0xe7, 0xff, 0xe7, 0xe7, 0xf3, 0x99, 0x99, 0xc3, 0x7e, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0xdc, 0x76, 0x00, 0xdc, 0x76, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x6e, 0xd8, 0xd8, 0xd8, 0xde, 0xd8, 0xd8, 0xd8, 0xd8, 0x6e, 0x00 },
    { 0x00, 0x00, 0x00, 0x6e, 0xdb, 0xd8, 0xdf, 0xdb, 0xdb, 0x6e, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x10, 0x38, 0x7c, 0xfe, 0x7c, 0x38, 0x10, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x08, 0x08, 0x08, 0x08, 0x3e, 0x00, 0x88, 0x88, 0xf8, 0x88, 0x88, 0x00 },
    { 0x00, 0x00, 0x20, 0x20, 0x38, 0x20, 0x3e, 0x00, 0x80, 0x80, 0xe0, 0x80, 0xf8, 0x00 },
    { 0x00, 0x00, 0x22, 0x24, 0x3e, 0x22, 0x3c, 0x00, 0x78, 0x80, 0x80, 0x80, 0x78, 0x00 },
    { 0x00, 0x00, 0x20, 0x20, 0x38, 0x20, 0x3e, 0x00, 0xf8, 0x80, 0x80, 0x80, 0x80, 0x00 },
    { 0x22, 0x88, 0x22, 0x88, 0x22, 0x88, 0x22, 0x88, 0x22, 0x88, 0x22, 0x88, 0x22, 0x88 },
    { 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa },
    { 0xee, 0xbb, 0xee, 0xbb, 0xee, 0xbb, 0xee, 0xbb, 0xee, 0xbb, 0xee, 0xbb, 0xee, 0xbb },
    { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff },
    { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff },
    { 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0 },
    { 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f },
    { 0x00, 0x00, 0x3e, 0x20, 0x20, 0x20, 0x20, 0x00, 0x88, 0x98, 0xa8, 0xc8, 0x88, 0x00 },
    { 0x00, 0x00, 0x08, 0x08, 0x08, 0x08, 0x3e, 0x00, 0x20, 0x50, 0x50, 0x88, 0x88, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x7e, 0x30, 0x18, 0x0c, 0x06, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x7e, 0x0c, 0x18, 0x30, 0x60, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0xc0, 0x60, 0xfe, 0x38, 0xfe, 0x0c, 0x06, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x02, 0x0e, 0x3e, 0x7e, 0xfe, 0x7e, 0x3e, 0x0e, 0x02, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x80, 0xe0, 0xf0, 0xfc, 0xfe, 0xfc, 0xf0, 0xe0, 0x80, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x7e, 0x3c, 0x18, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x18, 0x3c, 0x7e, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x0c, 0xfe, 0x0c, 0x18, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x60, 0xfe, 0x60, 0x30, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x18, 0x3c, 0x7e, 0x18, 0x18, 0x18, 0x7e, 0x3c, 0x18, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x6c, 0xfe, 0x6c, 0x28, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x60, 0xfe, 0x66, 0x36, 0x06, 0x06, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x6c, 0x6c, 0x6c, 0x6c, 0x6e, 0x7c, 0xc0, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x18, 0x18, 0x3c, 0x3c, 0x3c, 0x18, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x36, 0x36, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6c, 0xfe, 0x6c, 0x6c, 0xfe, 0x6c, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x10, 0x7c, 0xd6, 0x1c, 0x38, 0x70, 0xd6, 0x7c, 0x10, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0xc6, 0x66, 0x30, 0x18, 0x0c, 0x66, 0x62, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x76, 0xcc, 0xce, 0xf6, 0x76, 0x38, 0x38, 0x6c, 0x38, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x0c, 0x1c, 0x1c, 0x00 },
    { 0x00, 0x00, 0x00, 0x0c, 0x18, 0x30, 0x30, 0x30, 0x30, 0x30, 0x18, 0x0c, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x30, 0x18, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x18, 0x30, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x6c, 0x38, 0xfe, 0x38, 0x6c, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x7e, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x18, 0x0c, 0x0c, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0xc0, 0x60, 0x30, 0x18, 0x0c, 0x06, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xd6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x7e, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x78, 0x18, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0xfe, 0xc6, 0x60, 0x30, 0x18, 0x0c, 0xc6, 0xc6, 0x7c, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0x06, 0x06, 0x3c, 0x06, 0x06, 0xc6, 0x7c, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x0c, 0x0c, 0x0c, 0xfe, 0xcc, 0x6c, 0x3c, 0x1c, 0x0c, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0x06, 0x06, 0xfc, 0xc0, 0xc0, 0xc0, 0xfe, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xfc, 0xc0, 0xc0, 0xc6, 0x7c, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x30, 0x30, 0x30, 0x30, 0x30, 0x18, 0x0c, 0xc6, 0xfe, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0x7c, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0x06, 0x06, 0x7e, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x0c, 0x0c, 0x00, 0x00, 0x0c, 0x0c, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x18, 0x0c, 0x0c, 0x0c, 0x00, 0x00, 0x0c, 0x0c, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x0c, 0x18, 0x30, 0x60, 0xc0, 0x60, 0x30, 0x18, 0x0c, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x00, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x60, 0x30, 0x18, 0x0c, 0x06, 0x0c, 0x18, 0x30, 0x60, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x18, 0x18, 0x0c, 0xc6, 0xc6, 0x7c, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x7e, 0xc0, 0xdc, 0xde, 0xde, 0xde, 0xc6, 0xc6, 0x7c, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0xc6, 0xc6, 0xc6, 0xfe, 0xc6, 0xc6, 0xc6, 0x6c, 0x38, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0xfc, 0x66, 0x66, 0x66, 0x7c, 0x66, 0x66, 0x66, 0xfc, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x3c, 0x66, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0x66, 0x3c, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0xf8, 0x6c, 0x66, 0x66, 0x66, 0x66, 0x66, 0x6c, 0xf8, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0xfe, 0x66, 0x60, 0x60, 0x7c, 0x60, 0x60, 0x66, 0xfe, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0xf0, 0x60, 0x60, 0x60, 0x7c, 0x60, 0x60, 0x66, 0xfe, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xce, 0xc0, 0xc0, 0xc6, 0xc6, 0x7c, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0xc6, 0xc6, 0xc6, 0xc6, 0xfe, 0xc6, 0xc6, 0xc6, 0xc6, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x3c, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3c, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x70, 0xd8, 0xd8, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3c, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0xc6, 0xc6, 0xcc, 0xd8, 0xf0, 0xf0, 0xd8, 0xcc, 0xc6, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0xfe, 0x66, 0x62, 0x60, 0x60, 0x60, 0x60, 0x60, 0xf0, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0xc6, 0xc6, 0xd6, 0xd6, 0xd6, 0xfe, 0xee, 0xc6, 0xc6, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0xc6, 0xce, 0xce, 0xde, 0xf6, 0xe6, 0xe6, 0xc6, 0xc6, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0xf0, 0x60, 0x60, 0x60, 0x7c, 0x66, 0x66, 0x66, 0xfc, 0x00, 0x00 },
    { 0x00, 0x00, 0x06, 0x7c, 0xd6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0xe6, 0x66, 0x6c, 0x78, 0x7c, 0x66, 0x66, 0x66, 0xfc, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0x06, 0x0c, 0x38, 0x60, 0xc0, 0xc6, 0x7c, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x3c, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x5a, 0x7e, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x10, 0x38, 0x6c, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0xc6, 0xc6, 0xee, 0xfe, 0xd6, 0xd6, 0xd6, 0xc6, 0xc6, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0xc6, 0xc6, 0x6c, 0x38, 0x38, 0x38, 0x6c, 0xc6, 0xc6, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x3c, 0x18, 0x18, 0x18, 0x3c, 0x66, 0x66, 0x66, 0x66, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0xfe, 0xc6, 0xc2, 0x60, 0x30, 0x18, 0x8c, 0xc6, 0xfe, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x7c, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x7c, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x06, 0x0c, 0x18, 0x30, 0x60, 0xc0, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x7c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x7c, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x66, 0x3c, 0x18, 0x00, 0x00 },
    { 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x18, 0x1c, 0x1c, 0x00 },
    { 0x00, 0x00, 0x00, 0x76, 0xdc, 0xcc, 0x7c, 0x0c, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0xfc, 0x66, 0x66, 0x66, 0x66, 0x7c, 0x60, 0x60, 0xe0, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc0, 0xc0, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x7e, 0xcc, 0xcc, 0xcc, 0xcc, 0x7c, 0x0c, 0x0c, 0x1c, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc0, 0xfe, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x78, 0x30, 0x30, 0x30, 0xfc, 0x30, 0x30, 0x36, 0x1c, 0x00, 0x00 },
    { 0x00, 0x7c, 0xc6, 0x06, 0x7e, 0xc6, 0xc6, 0xce, 0x76, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0xe6, 0x66, 0x66, 0x66, 0x76, 0x6c, 0x60, 0x60, 0xe0, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x3c, 0x18, 0x18, 0x18, 0x18, 0x38, 0x00, 0x18, 0x18, 0x00, 0x00 },
    { 0x00, 0x78, 0xcc, 0xcc, 0x0c, 0x0c, 0x0c, 0x0c, 0x1c, 0x00, 0x0c, 0x0c, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0xe6, 0x66, 0x6c, 0x78, 0x6c, 0x66, 0x60, 0x60, 0xe0, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x3c, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x38, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0xc6, 0xc6, 0xd6, 0xd6, 0xfe, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x66, 0x66, 0x66, 0x66, 0x66, 0xdc, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0xf0, 0x60, 0x60, 0x7c, 0x66, 0x66, 0x66, 0xdc, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x1e, 0x0c, 0x0c, 0x7c, 0xcc, 0xcc, 0xcc, 0x76, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0xf0, 0x60, 0x60, 0x60, 0x66, 0xdc, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0x1c, 0x70, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x1c, 0x36, 0x30, 0x30, 0x30, 0xfc, 0x30, 0x30, 0x30, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x76, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x10, 0x38, 0x6c, 0xc6, 0xc6, 0xc6, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x6c, 0xfe, 0xd6, 0xd6, 0xc6, 0xc6, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0xc6, 0x6c, 0x38, 0x38, 0x6c, 0xc6, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x7c, 0xc6, 0x06, 0x76, 0xce, 0xc6, 0xc6, 0xc6, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0xfe, 0x62, 0x30, 0x18, 0x8c, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x0e, 0x18, 0x18, 0x18, 0x70, 0x18, 0x18, 0x18, 0x0e, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x70, 0x18, 0x18, 0x18, 0x0e, 0x18, 0x18, 0x18, 0x70, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdc, 0x76, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x3c, 0x18, 0x18, 0x3c, 0x66, 0x66, 0x66, 0x00, 0x66, 0x66, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18 },
    { 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18 },
    { 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x1f, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18 },
    { 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0xf8, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18 },
    { 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0xff, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18 },
    { 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x30, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x60, 0x6f, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c },
    { 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c },
    { 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x6f, 0x60, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x6f, 0x60, 0x6f, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x0c, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x0c, 0xec, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0xef, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c },
    { 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0xec, 0x0c, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0xec, 0x0c, 0xec, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c },
    { 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0xef, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0xef, 0x00, 0xef, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c },
    { 0x00, 0x00, 0xfe, 0x82, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x18, 0x3c, 0x3c, 0x3c, 0x18, 0x18, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x10, 0x7c, 0xd6, 0xd0, 0xd0, 0xd6, 0x7c, 0x10, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x6c, 0xf6, 0x66, 0x60, 0xf0, 0x60, 0x60, 0x6c, 0x38, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x3c, 0x62, 0x60, 0xf8, 0x60, 0xf8, 0x60, 0x62, 0x3c, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x18, 0x18, 0x3c, 0x18, 0x7e, 0x18, 0x3c, 0x66, 0x66, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0x06, 0x1c, 0x60, 0xc6, 0xc6, 0x7c, 0x00, 0x38, 0x6c },
    { 0x00, 0x7c, 0xc6, 0xc6, 0x0c, 0x7c, 0xc6, 0xc6, 0x7c, 0x60, 0xc6, 0xc6, 0x7c, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x7e, 0x81, 0x99, 0xa5, 0xa1, 0xa5, 0x99, 0x81, 0x7e, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x3e, 0x6c, 0x6c, 0x3c, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x6c, 0xd8, 0x6c, 0x36, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x06, 0x06, 0x06, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x7e, 0x81, 0xa5, 0xa5, 0xb9, 0xa5, 0xb9, 0x81, 0x7e, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x6c, 0x38, 0x00 },
    { 0x00, 0x00, 0x00, 0x7e, 0x00, 0x18, 0x18, 0x7e, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x30, 0x18, 0x6c, 0x38, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x6c, 0x18, 0x6c, 0x38, 0x00 },
    { 0x00, 0x00, 0x00, 0xfe, 0xc6, 0x62, 0x30, 0x18, 0x8c, 0xc6, 0xfe, 0x00, 0x38, 0x6c },
    { 0xc0, 0xc0, 0xc0, 0xf6, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x1b, 0x1b, 0x1b, 0x1b, 0x7b, 0xdb, 0xdb, 0xdb, 0x7f, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0xfe, 0x62, 0x30, 0x18, 0x8c, 0xfe, 0x00, 0x38, 0x6c, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x30, 0x30, 0x70, 0x30, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x00, 0x38, 0x6c, 0x6c, 0x38, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0xd8, 0x6c, 0x36, 0x6c, 0xd8, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x77, 0xdf, 0xd8, 0xd8, 0xde, 0xde, 0xd8, 0xdf, 0x77, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x6e, 0xdb, 0xd8, 0xdf, 0xdb, 0x6e, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x3c, 0x18, 0x18, 0x18, 0x3c, 0x66, 0x66, 0x00, 0x66, 0x66, 0x00 },
    { 0x00, 0x7c, 0xc6, 0xc6, 0x60, 0x30, 0x30, 0x00, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0xc6, 0xc6, 0xfe, 0xc6, 0xc6, 0x6c, 0x38, 0x00, 0x18, 0x30, 0x60 },
    { 0x00, 0x00, 0x00, 0xc6, 0xc6, 0xfe, 0xc6, 0xc6, 0x6c, 0x38, 0x00, 0x30, 0x18, 0x0c },
    { 0x00, 0x00, 0x00, 0xc6, 0xc6, 0xfe, 0xc6, 0xc6, 0x6c, 0x38, 0x00, 0x6c, 0x38, 0x10 },
    { 0x00, 0x00, 0x00, 0xc6, 0xc6, 0xfe, 0xc6, 0xc6, 0x6c, 0x38, 0x00, 0xdc, 0x76, 0x00 },
    { 0x00, 0x00, 0x00, 0xc6, 0xc6, 0xfe, 0xc6, 0xc6, 0x6c, 0x38, 0x00, 0x6c, 0x6c, 0x00 },
    { 0x00, 0x00, 0x00, 0xc6, 0xc6, 0xfe, 0xc6, 0xc6, 0x6c, 0x38, 0x00, 0x38, 0x6c, 0x38 },
    { 0x00, 0x00, 0x00, 0xde, 0xd8, 0xd8, 0xd8, 0xfe, 0xd8, 0xd8, 0xd8, 0xd8, 0x7e, 0x00 },
    { 0x38, 0x6c, 0x18, 0x3c, 0x66, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0x66, 0x3c, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0xfe, 0x66, 0x60, 0x7c, 0x60, 0x66, 0xfe, 0x00, 0x0c, 0x18, 0x30 },
    { 0x00, 0x00, 0x00, 0xfe, 0x66, 0x60, 0x7c, 0x60, 0x66, 0xfe, 0x00, 0x30, 0x18, 0x0c },
    { 0x00, 0x00, 0x00, 0xfe, 0x66, 0x60, 0x7c, 0x60, 0x66, 0xfe, 0x00, 0x6c, 0x38, 0x10 },
    { 0x00, 0x00, 0x00, 0xfe, 0x66, 0x60, 0x7c, 0x60, 0x66, 0xfe, 0x00, 0x6c, 0x6c, 0x00 },
    { 0x00, 0x00, 0x00, 0x3c, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3c, 0x00, 0x0c, 0x18, 0x30 },
    { 0x00, 0x00, 0x00, 0x3c, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3c, 0x00, 0x30, 0x18, 0x0c },
    { 0x00, 0x00, 0x00, 0x3c, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3c, 0x00, 0x66, 0x3c, 0x18 },
    { 0x00, 0x00, 0x00, 0x3c, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3c, 0x00, 0x66, 0x66, 0x00 },
    { 0x00, 0x00, 0x00, 0xf8, 0x6c, 0x66, 0x66, 0xf6, 0x66, 0x66, 0x6c, 0xf8, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0xc6, 0xc6, 0xce, 0xde, 0xf6, 0xe6, 0xc6, 0x00, 0xdc, 0x76, 0x00 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x18, 0x30, 0x60 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x30, 0x18, 0x0c },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x6c, 0x38, 0x10 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0xdc, 0x76, 0x00 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x6c, 0x6c, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x6c, 0x38, 0x38, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0xfc, 0xc6, 0xe6, 0xf6, 0xd6, 0xde, 0xce, 0xc6, 0x7e, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x00, 0x18, 0x30, 0x60 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x00, 0x30, 0x18, 0x0c },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x00, 0x6c, 0x38, 0x10 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x00, 0x6c, 0x6c, 0x00 },
    { 0x00, 0x00, 0x00, 0x3c, 0x18, 0x18, 0x3c, 0x66, 0x66, 0x66, 0x00, 0x18, 0x0c, 0x06 },
    { 0x00, 0x00, 0x00, 0xf0, 0x60, 0x7c, 0x66, 0x66, 0x66, 0x7c, 0x60, 0xf0, 0x00, 0x00 },
    { 0x00, 0x00, 0x80, 0xdc, 0xd6, 0xc6, 0xc6, 0xcc, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x76, 0xdc, 0xcc, 0x7c, 0x0c, 0x78, 0x00, 0x18, 0x30, 0x60, 0x00 },
    { 0x00, 0x00, 0x00, 0x76, 0xdc, 0xcc, 0x7c, 0x0c, 0x78, 0x00, 0x60, 0x30, 0x18, 0x00 },
    { 0x00, 0x00, 0x00, 0x76, 0xdc, 0xcc, 0x7c, 0x0c, 0x78, 0x00, 0xcc, 0x78, 0x30, 0x00 },
    { 0x00, 0x00, 0x00, 0x76, 0xdc, 0xcc, 0x7c, 0x0c, 0x78, 0x00, 0xdc, 0x76, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x76, 0xdc, 0xcc, 0x7c, 0x0c, 0x78, 0x00, 0x6c, 0x6c, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x76, 0xdc, 0xcc, 0x7c, 0x0c, 0x78, 0x00, 0x38, 0x6c, 0x38, 0x00 },
    { 0x00, 0x00, 0x00, 0x7e, 0xdb, 0xd8, 0x7f, 0x1b, 0xdb, 0x7e, 0x00, 0x00, 0x00, 0x00 },
    { 0x38, 0x6c, 0x18, 0x7c, 0xc6, 0xc0, 0xc0, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc0, 0xfe, 0xc6, 0x7c, 0x00, 0x0c, 0x18, 0x30, 0x00 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc0, 0xfe, 0xc6, 0x7c, 0x00, 0x30, 0x18, 0x0c, 0x00 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc0, 0xfe, 0xc6, 0x7c, 0x00, 0x6c, 0x38, 0x10, 0x00 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc0, 0xfe, 0xc6, 0x7c, 0x00, 0x6c, 0x6c, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x3c, 0x18, 0x18, 0x18, 0x18, 0x38, 0x00, 0x18, 0x30, 0x60, 0x00 },
    { 0x00, 0x00, 0x00, 0x3c, 0x18, 0x18, 0x18, 0x18, 0x38, 0x00, 0x30, 0x18, 0x0c, 0x00 },
    { 0x00, 0x00, 0x00, 0x3c, 0x18, 0x18, 0x18, 0x18, 0x38, 0x00, 0x66, 0x3c, 0x18, 0x00 },
    { 0x00, 0x00, 0x00, 0x3c, 0x18, 0x18, 0x18, 0x18, 0x38, 0x00, 0x6c, 0x6c, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0x7e, 0x0c, 0x78, 0x30, 0x78, 0x00 },
    { 0x00, 0x00, 0x00, 0x66, 0x66, 0x66, 0x66, 0x66, 0xdc, 0x00, 0xdc, 0x76, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x18, 0x30, 0x60, 0x00 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x30, 0x18, 0x0c, 0x00 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x6c, 0x38, 0x10, 0x00 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0xdc, 0x76, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x6c, 0x6c, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x7e, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0xfc, 0xe6, 0xf6, 0xde, 0xce, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00 },
    { 0x00, 0x00, 0x00, 0x76, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0x00, 0x30, 0x60, 0xc0, 0x00 },
    { 0x00, 0x00, 0x00, 0x76, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0x00, 0x30, 0x18, 0x0c, 0x00 },
    { 0x00, 0x00, 0x00, 0x76, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0x00, 0xcc, 0x78, 0x30, 0x00 },
    { 0x00, 0x00, 0x00, 0x76, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0x00, 0xcc, 0xcc, 0x00, 0x00 },
    { 0x00, 0x7c, 0xc6, 0x06, 0x76, 0xce, 0xc6, 0xc6, 0xc6, 0x00, 0x30, 0x18, 0x0c, 0x00 },
    { 0xf0, 0x60, 0x60, 0x78, 0x6c, 0x6c, 0x6c, 0x78, 0x60, 0x60, 0xf0, 0x00, 0x00, 0x00 },
    { 0x00, 0x7c, 0xc6, 0x06, 0x76, 0xce, 0xc6, 0xc6, 0xc6, 0x00, 0xc6, 0xc6, 0x00, 0x00 }
};

#define LEN 8192  //  Maximum length of text string
void PrintSDL(const char* format , ...){
    /* Convenience routine to output raster text
     * Use VARARGS to make this more flexible
     * Author: Willem A. (Vlakkies) Schreüder  */
   char    buf[LEN];
   va_list args;
   //  Turn the parameters into a character string
   va_start(args,format);
   vsnprintf(buf,LEN,format,args);
   va_end(args);
   //  Setup font as display lists on first use
   if (!font)
   {
      int i;
      glPixelStorei(GL_UNPACK_ALIGNMENT,1);
      font = glGenLists(256);
      for (i=0;i<256;i++)
      {
         glNewList(font+i,GL_COMPILE);
         glBitmap(8,14,0.0,0.0,9.0,0.0,letters[i]);
         glEndList();
      }
   }
   //  Display the characters at the current raster position
   glListBase(font);
   glCallLists(strlen(buf),GL_UNSIGNED_BYTE,(GLubyte *)buf);
}

// ___ End Func ____________________________________________________________________________________________________________________________




/* === Spare Parts =========================================================================================================================



   ___ End Parts ___________________________________________________________________________________________________________________________

*/

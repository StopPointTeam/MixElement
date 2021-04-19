/*
MemoryUsage.h - MemoryUsage library V2.10
Copyright (c) 2015 Thierry Paris.  All right reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef __MemoryUsage_h__
#define __MemoryUsage_h__

#include <stdint.h>
#include <NeoHWSerial.h>

#define NeoSerialDebug NeoSerial3

/*! \mainpage

A full explanation in french can be read at http://www.locoduino.org/ecrire/?exec=article&action=redirect&type=article&id=149 .

Roughly, the SRAM memory is divided into four areas: the static data, the heap, the free ram and the stack.

The static data size is given by the compiler itself after the building. this is filled by all variables and 
arrays declared in global scope, or with 'static' keyword.

The heap is filled with all the dynamic allocations done with 'new' keyword or 'malloc' functions.

The stack start from the end of the SRAM area and grow and shrink downward at each function call, it stores 
all the local data internal to a function, function arguments (depending of the architecture, arguments can be 
stored in CPU registers to improve speed...) , and addresses for function returns to caller.

SRAM memory 
\verbatim
+---------------+------------------+---------------------------------------------+-----------------+
|               |                  |                                             |                 |
|               |                  |                                             |                 |
|    static     |                  |                                             |                 |
|     data      |       heap       |                   free ram                  |      stack      |
|               |                  |                                             |                 |
|               |                  |                                             |                 |
|               |                  |                                             |                 |
+---------------+------------------+---------------------------------------------+-----------------+
       _end or __heap_start     __brkval                                         SP             RAMEND
\endverbatim

Source : http://www.nongnu.org/avr-libc/user-manual/malloc.html

MemoryUsage try to help you to find the actual memory status with differents strategies, but dont forget
that when you observe something, you change the result of the observation : execution time is consumed
by the analysis tools, and memory used will grow because of these tools !

1. First, there are the MACROs to show memory areas start/end addresses and actual sizes.

2. Second, there is a display function to show a 'map' of the memory...

3. Third, a function can give you the current size of the free ram using a stack tag, which is more accurate 
than the MACRO.

4. Fourth, an elegant way to try to understand how much size has reached the stack during execution. 
It will 'decorate' the internal memory, and try to identify after a moment of execution at what place
the first byte of the memory is not anymore decorated...
The function mu_StackPaint will be called _before the setup() function of your sketch, to 'paint' or 
'decorate' all the bytes of the SRAM momery with a particular code, called the CANARY... Later, a function
mu_StackCount can be called to get the actual maximum size reached by the stack by counter the byte
no more painted.
This is a copy / adaptation of the library StackPaint available here : https://github.com/WickedDevice/StackPaint

5. And five at least, and because a stack grow and shrink continuously, the macros STACK_DECLARE / STACK_COMPUTE / STACK_PRINT
try to get the greatest size of the stack by 'sampling' the execution.
Start your code by

    #include <MemoryUsage.h>

    STACK_DECLARE

    void setup()
    ...

then add a STACK_COMPUTE in any function that can be called :

    void subFonction()
    {
        double v[SIZE];
        STACK_COMPUTE;
 
        .... // do things
    }

and finish by printing on the console the biggest size of the stack with STACK_PRINT or STACK_PRINT_TEXT.
Be careful with this method, this introduce  some code in every function of your sketch, so if the timing
is important for your applicaion, take care of it !
*/

/*! \file MemoryUsage.h

Main library header file.
*/

extern uint8_t _end;
extern uint8_t __stack;
extern uint8_t *__brkval;
extern uint8_t *__data_start;
extern uint8_t *__data_end;
extern uint8_t *__heap_start;
extern uint8_t *__heap_end;
extern uint8_t *__bss_start;
extern uint8_t *__bss_end;

//
// Memory addresses
//

/// Print data start on NeoSerialDebug console.
#define MEMORY_PRINT_START		{ NeoSerialDebug.print(F("Data start:")); NeoSerialDebug.println((int) &__data_start); }
/// Print data end / heap start on NeoSerialDebug console.
#define MEMORY_PRINT_HEAPSTART	{ NeoSerialDebug.print(F("Heap start:")); NeoSerialDebug.println((int)&__heap_start); }
/// Print heap end / free ram area on NeoSerialDebug console.
#define MEMORY_PRINT_HEAPEND	{ NeoSerialDebug.print(F("Heap end:")); NeoSerialDebug.println(__brkval == 0 ? (int)&__heap_start : (int)__brkval); }
/// Print free ram end / stack start on NeoSerialDebug console.
#define MEMORY_PRINT_STACKSTART	{ NeoSerialDebug.print(F("Stack start:")); NeoSerialDebug.println((int) SP); }
/// Print end of memory on NeoSerialDebug console.
#define MEMORY_PRINT_END		{ NeoSerialDebug.print(F("Stack end:")); NeoSerialDebug.println((int) RAMEND); }

/// Print heap size on NeoSerialDebug console.
#define MEMORY_PRINT_HEAPSIZE	{ NeoSerialDebug.print(F("Heap size:")); NeoSerialDebug.println((int) (__brkval == 0 ? (int)&__heap_start : (int)__brkval) - (int)&__heap_start); }
/// Print stack size on NeoSerialDebug console.
#define MEMORY_PRINT_STACKSIZE	{ NeoSerialDebug.print(F("Stack size:")); NeoSerialDebug.println((int) RAMEND - (int)SP); }
/// Print free ram size on NeoSerialDebug console.
#define MEMORY_PRINT_FREERAM	{ NeoSerialDebug.print(F("Free ram:")); NeoSerialDebug.println((int) SP - (int) (__brkval == 0 ? (int)&__heap_start : (int)__brkval)); }
/// Print total SRAM size on NeoSerialDebug console.
#define MEMORY_PRINT_TOTALSIZE	{ NeoSerialDebug.print(F("SRAM size:")); NeoSerialDebug.println((int) RAMEND - (int) &__data_start); }

/// Displays the 'map' of the current state of the Arduino's SRAM memory on the NeoSerialDebug console.
void SRamDisplay(void);

//
// Stack count part. STACK_COMPUTE will get the maximum size of the stack at the moment...
//

/// Must be used only one time, outside any function.
#define STACK_DECLARE      unsigned int mu_stack_size = (RAMEND - SP);

/// Must be called to update the current maximum size of the stack, at each function beginning.
#define STACK_COMPUTE      { mu_stack_size = (RAMEND - SP) > mu_stack_size ? (RAMEND - SP) : mu_stack_size;}

/// Compute the current maximum and show it now with customized text.
#define STACK_PRINT_TEXT(text)  { STACK_COMPUTE; NeoSerialDebug.print(text);  NeoSerialDebug.println(mu_stack_size); }

/// Compute the current maximum and show it now with default text.
#define STACK_PRINT        STACK_PRINT_TEXT(F("Stack Maximum Size (Instrumentation method): "));

//
// Free Ram part.
//

/// Shows the current free SRAM memory with customized text.
#define FREERAM_PRINT_TEXT(text)  NeoSerialDebug.print(text);  NeoSerialDebug.println(mu_freeRam());

/// Shows the current free SRAM memory with default text.
#define FREERAM_PRINT      FREERAM_PRINT_TEXT(F("Free Ram Size: "));

/// Show the free Ram size on console.
int mu_freeRam(void);

//
// StackPaint part. This macro gives a view of used stack area at the end of execution...
//

/// Show the stack size on console.
uint16_t mu_StackCount(void);

/// Compute the current maximum and show it now with customized text.
#define STACKPAINT_PRINT_TEXT(text)  { NeoSerialDebug.print(text);  NeoSerialDebug.println(mu_StackCount()); }

/// Compute the current maximum and show it now with default text.
#define STACKPAINT_PRINT        STACKPAINT_PRINT_TEXT(F("Stack Maximum Size (Painting method): "));


#endif

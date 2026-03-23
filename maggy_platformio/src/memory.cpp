#include "memory.h"

#include <Arduino.h>

#define printf Serial.printf

void memInfo () {
    constexpr auto RAM_BASE   = 0x2020'0000;
    constexpr auto RAM_SIZE   = 512 << 10;
    constexpr auto FLASH_BASE = 0x6000'0000;
#if ARDUINO_TEENSY40
    constexpr auto FLASH_SIZE = 2 << 20;
#elif ARDUINO_TEENSY41
    constexpr auto FLASH_SIZE = 8 << 20;
#endif

    // note: these values are defined by the linker, they are not valid memory
    // locations in all cases - by defining them as arrays, the C++ compiler
    // will use the address of these definitions - it's a big hack, but there's
    // really no clean way to get at linker-defined symbols from the .ld file

    extern char* __brkval;
    extern char _stext[], _etext[], _sbss[], _ebss[], _sdata[], _edata[],
        _estack[], _heap_start[], _heap_end[], _itcm_block_count[];

    auto sp = (char*) __builtin_frame_address(0);

//     printf("_stext        %08x\n",      _stext);
//     printf("_etext        %08x +%db\n", _etext, _etext-_stext);
//     printf("_sdata        %08x\n",      _sdata);
//     printf("_edata        %08x +%db\n", _edata, _edata-_sdata);
//     printf("_sbss         %08x\n",      _sbss);
//     printf("_ebss         %08x +%db\n", _ebss, _ebss-_sbss);
//     printf("curr stack    %08x +%db\n", sp, sp-_ebss);
//     printf("_estack       %08x +%db\n", _estack, _estack-sp);
//     printf("_heap_start   %08x\n",      _heap_start);
//     printf("__brkval      %08x +%db\n", __brkval, __brkval-_heap_start);
//     printf("_heap_end     %08x +%db\n", _heap_end, _heap_end-__brkval);
// #if ARDUINO_TEENSY41
//     extern char _extram_start[], _extram_end[],*__brkval;
//     printf("_extram_start %08x\n",      _extram_start);
//     printf("_extram_end   %08x +%db\n", _extram_end,
//                                                _extram_end-_extram_start);
// #endif
//     printf("\n");

//     printf("<ITCM>  %08x .. %08x\n",
//             _stext, _stext + ((int) _itcm_block_count << 15) - 1);
//     printf("<DTCM>  %08x .. %08x\n",
//             _sdata, _estack - 1);
//     printf("<RAM>   %08x .. %08x\n",
//             RAM_BASE, RAM_BASE + RAM_SIZE - 1);
//     printf("<FLASH> %08x .. %08x\n",
//             FLASH_BASE, FLASH_BASE + FLASH_SIZE - 1);
// #if ARDUINO_TEENSY41
//     extern uint8_t external_psram_size;
//     if (external_psram_size > 0)
//         printf("<PSRAM> %08x .. %08x\n",
//             _extram_start, _extram_start + (external_psram_size<<20) - 1);
// #endif
//     printf("\n");

    auto stack = sp-_ebss;
    printf("avail STACK %8d b %5d kb\n", stack, stack>>10);

    auto heap = _heap_end-__brkval;
    printf("avail HEAP  %8d b %5d kb\n", heap, heap>>10);

// #if ARDUINO_TEENSY41
//     auto psram = _extram_start + (external_psram_size<<20) - _extram_end;
//     printf("avail PSRAM %8d b %5d kb\n", psram, psram>>10);
// #endif
}
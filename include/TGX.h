//
// Created by ctlf on 1/8/25.
//

#ifndef TGX_H
#define TGX_H

/*
	TGX Implementation complient with TGX spec Version 1 Revision 2
	Implementation conforms to the minimum operating standards
	as specified in the spec. This implementation, currently does
	not vary from the spec but may in the future. Any variations
	will be listed
	* Here:
	*
	This software is provided "As Is", has no warrenty and the
	developer shall not be held reliable for any damages caused
	to the end user's computer for any reason. This system is
	developed with safety and performance in mind, but no guarentees
	are provided.
	Installing, distributing, or using this software in any way is
	an implicit consent to these terms.
*/

#define TGX_IMPL_VERSION "0.0.1"


#include <SDL2/SDL.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

#define TGX_ENUM int

// 32 bit register enum
#define REG_RA 0x00
#define REG_RB 0x01
#define REG_RC 0x02
#define REG_RD 0x03
#define REG_RE 0x04
#define REG_RF 0x05
#define REG_RW 0x06
#define REG_RX 0x07
#define REG_RY 0x08
#define REG_RZ 0x09
#define REG_IP 0x0A
#define REG_JR 0x0B
#define REG_SB 0x0C
#define REG_SP 0x0D
#define REG_SH 0x0E
#define REG_PF 0x0F
#define REG_RI 0x10
#define REG_RJ 0x11
#define REG_RK 0x12
#define REG_RL 0x13
#define REG32_COUNT 0x14

// 16 bit register enum
#define REG_RAL 0x00
#define REG_RAH 0x01
#define REG_RBL 0x02
#define REG_RBH 0x03
#define REG_RCL 0x04
#define REG_RCH 0x05
#define REG_RDL 0x06
#define REG_RDH 0x07
#define REG_REL 0x08
#define REG_REH 0x09
#define REG_RFL 0x0A
#define REG_RFH 0x0B
#define REG_RWL 0x0C
#define REG_RWH 0x0D
#define REG_RXL 0x0E
#define REG_RXH 0x0F
#define REG_RYL 0x10
#define REG_RYH 0x11
#define REG_RZL 0x12
#define REG_RZH 0x13
#define REG16_COUNT 0x14

// 8 bit register indices correspond EXACTLY wwith the 
// 32 bit registers they take part of. They are limited to
// RAS through RZS however.
#define REG8_COUNT 0x0A
#define REG64_COUNT 0x02

#define REG_FX0 0x00
#define REG_FY0 0x01
#define REG_FZ0 0x02
#define REG_FW0 0x03
#define REG_FX1 0x04
#define REG_FY1 0x05
#define REG_FZ1 0x06
#define REG_FW1 0x07

#define REGF32_COUNT 0x08
#define REGVEC4_COUNT 0x02

#define INT_TABLE_COUNT 256

#define KILOBYTES(n) ((n) * 1024)

#define ROM_SIZE               KILOBYTES( 32)
#define PROGRAM_CACHE_SIZE     KILOBYTES( 64)
#define GRAPHICS_CACHE_SIZE    KILOBYTES( 64)
#define GRAPHICS_QUEUE_SIZE    KILOBYTES(  8)
#define SHARED_RAM_SIZE        KILOBYTES(480)
#define INSTRUCTION_CACHE_SIZE KILOBYTES(512)

#define PRINCIPLE_MEMORY_SIZE (ROM_SIZE + PROGRAM_CACHE_SIZE + \
		GRAPHICS_CACHE_SIZE + GRAPHICS_QUEUE_SIZE + \
		SHARED_RAM_SIZE + INSTRUCTION_CACHE_SIZE)

#define TGX_SUCCESS 0
#define TGX_ERROR  -1

#define TGX_PF_Flag_ErrorMask    0x000000FF
#define TGX_PF_Flag_ErrorBit     0x00000100
#define TGX_PF_Flag_SysActiveBit 0x00000200
#define TGX_PF_Flag_ZeroBit      0x00000400
#define TGX_PF_Flag_NegBit       0x00000800
#define TGX_PF_Flag_GraphicsFree 0x00001000

// UPDATE THIS IF ANY NEW INSTRUCTIONS ARE ADDED!!!!!
#define TGX_OPCODE_COUNT 0x011E

typedef struct {
	uint8_t* memory_begin;
	uint8_t* memory_end;
	uint8_t* rom_begin;
	uint8_t* rom_end;
	uint8_t* program_cache_begin;
	uint8_t* program_cache_end;
	uint8_t* graphics_cache_begin;
	uint8_t* graphics_cache_end;
	uint8_t* graphics_queue_begin;
	uint8_t* graphics_queue_end;
	uint8_t* shared_ram_begin;
	uint8_t* shared_ram_end;
	uint8_t* instruction_cache_begin;
	uint8_t* instruction_cache_end;
} PrincipleMemory;

int alloc_principle_memory(PrincipleMemory*);
void free_principle_memory(PrincipleMemory*);


typedef struct {
    union {
        uint32_t full;
        uint16_t half[2];
        uint8_t  single[4];
    };
} Reg32;

typedef struct {
    union {
        uint64_t full;
        uint32_t half[2];
    };
} Reg64;

typedef struct {
    float xyzw[4];
} Vec4;

typedef struct {
    Reg32 gp32[REG32_COUNT];
    Reg64 gp64[REG64_COUNT];
    Vec4 fpvec[REGVEC4_COUNT];
    uint64_t int_table[INT_TABLE_COUNT];
} ProgramThread;

typedef struct {

} GraphicsThread;

typedef struct {
    ProgramThread PU;
    GraphicsThread GU;
    PrincipleMemory Memory;
} TGXContext;

typedef struct {
	union {
		uint64_t full_value;
		struct {
			uint16_t opcode;
			union {
				uint16_t param16;
				uint8_t params[2];
			};
			union {
				uint8_t ext_params[4];
				uint32_t const_i32;
				float const_f32;
			};
		};
	};
} Instruction;

int init_program_thread(ProgramThread*);
void destroy_program_thread(ProgramThread*);
int init_graphics_thread(GraphicsThread*);
void destroy_graphics_thread(GraphicsThread*);

int program_thread_exec(TGXContext*);

#endif //TGX_H

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
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <SDL2/SDL_thread.h>

#include <time.h>


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

// 8 bit register indices correspond EXACTLY with the
// 32 bit registers they take part of. They are limited to
// RAS through RZS.
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
#define REG_FX2 0x08
#define REG_FY2 0x09
#define REG_FZ2 0x0A
#define REG_FW2 0x0B
#define REG_FX3 0x0C
#define REG_FY3 0x0D
#define REG_FZ3 0x0E
#define REG_FW3 0x0F

#define REGF32_COUNT 0x10
#define REGVEC4_COUNT 0x04

#define INT_TABLE_COUNT 256

#define INT_RES_UNEXPECTED_ERROR 255
#define INT_RES_GQ_FULL          254
#define INT_RES_GQ_OVERFLOW      253


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

#define DEFAULT_STACK_SIZE KILOBYTES(2)

#define TGX_SUCCESS 0
#define TGX_ERROR  -1

#define MODE8 0
#define MODE16 1
#define MODE32 2
#define MODE64 3

#define TGX_PF_Flag_ErrorMask    0x000000FF
#define TGX_PF_Flag_ErrorBit     0x00000100
#define TGX_PF_Flag_SysActiveBit 0x00000200
#define TGX_PF_Flag_ZeroBit      0x00000400
#define TGX_PF_Flag_NegBit       0x00000800
#define TGX_PF_Flag_GraphicsFree 0x00001000

#define TGX_PF_Shift_ErrorBit 8
#define TGX_PF_Shift_SysActiveBit 9
#define TGX_PF_Shift_ZeroBit 10
#define TGX_PF_Shift_NegBit 11
#define TGX_PF_Shift_GraphicsFree 12

#define TGX_JMP_ELSE_FLAG 0x10
#define TGX_JMP_USE_REG_FLAG 0x01

// UPDATE THIS IF ANY NEW INSTRUCTIONS ARE ADDED!!!!!
#define TGX_OPCODE_COUNT 0x011F

#define TGX_EXIT_CODE_NONE 0
#define TGX_EXIT_CODE_RESTART 65535


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
	union {
		Reg32 gp32[REG32_COUNT];
		uint16_t gp16[REG32_COUNT * 2];
	};
	Reg64 gp64[REG64_COUNT];
	union {
		Vec4 vec[REGVEC4_COUNT];
		float f32[REGVEC4_COUNT * 4];
	};
    uint32_t int_table[INT_TABLE_COUNT];
} ProgramThread;

typedef struct {
	SDL_Window* window;
	SDL_Surface* appsurf;
} GraphicsContext;

#define GPU_ERR_NONE 0x00
#define GPU_ERR_INIT 0x01
#define GPU_ERR_NULL_DISPLAY 0x02
#define GPU_ERR_STACK_OVERFLOW 0x03
#define GPU_ERR_REINIT 0x04
#define GPU_ERR_UNKNOWN_CMD 0x05

typedef struct {

	GraphicsContext context;
	void* command_buffer_begin;
	void* command_buffer_end;
	void* command_buffer_cursor;
	void* shared_ram_begin;
	void* shared_ram_end;
	void* graphics_cache_begin;
	void* graphics_cache_end;
	void* error_buffer_begin;
	void* error_buffer_end;
	void* error_buffer_cursor;
	void* memory_begin;
	bool ready;
	bool shutdown_flag;
	SDL_mutex* mutex;
	SDL_cond* cond;
	SDL_Thread* thread;
} GraphicsThread;

typedef uint32_t(*TGX_swi_handler)(void* tgxContext, uint32_t code);

typedef struct {
    ProgramThread PU;
    GraphicsThread GU;
    PrincipleMemory Memory;
	TGX_swi_handler SWIFunc;
	int ExitCode;
	FILE* debug_trace;
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
				uint16_t ext_params16[2];
				uint32_t const_i32;
				float const_f32;
			};
		};
	};
} Instruction;

int init_program_thread(ProgramThread*);
void destroy_program_thread(ProgramThread*);
int init_graphics_thread(GraphicsThread*, PrincipleMemory* memBase);
void destroy_graphics_thread(GraphicsThread*);

int program_thread_exec(TGXContext*);

void TriggerGraphics(GraphicsThread* gu);
void fPrintInst(FILE* f, Instruction* inst);

#endif //TGX_H

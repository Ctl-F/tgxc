#include "TGX.h"

int load_rom(TGXContext* context);

uint32_t swi_handler(void* context, uint32_t code);

int main(void) {
    TGXContext context = {0};

    if (alloc_principle_memory(&context.Memory) != TGX_SUCCESS) {
        printf("Unable to allocate required memory to start the vm. \n");
        return 1;
    }

    init_program_thread(&context.PU);
    init_graphics_thread(&context.GU);

    if (load_rom(&context) != TGX_SUCCESS) {
        return 1;
    }

    context.SWIFunc = swi_handler;

    program_thread_exec(&context);

    return 0;
}
// Things officially work. In that the baseline instruction set is good and needs to be tested.
// Next step is make an assembler and test the instructions

int load_rom(TGXContext* ctx) {
    //TODO: load from file

    Instruction* h = (Instruction*)ctx->Memory.rom_begin;

    h->opcode = 0x000F;
    h->params[0] = 0x00;
    h->const_i32 = 2;
    h++;

    h->opcode = 0x000F;
    h->params[0] = 0x01;
    h->const_i32 = 3;
    h++;

    h->opcode = 0x0045;
    h->params[0] = 0x02;
    h->params[1] = 0x00;
    h->ext_params[0] = 0x01;
    h++;

    h->opcode = 0x0115;
    h->const_i32 = 0;
    h++;

    h->opcode = 0x010C;

    return TGX_SUCCESS;
}

uint32_t swi_handler(void* context, uint32_t code) {
    TGXContext* ctx = context;

    switch (code) {
        case 0x00000000:
            for (int i=0; i< REG32_COUNT; i++) {
                printf("%2X: %8X (%u)\n", i, ctx->PU.gp32[i].full, ctx->PU.gp32[i].full);
            }
            break;
        default:
            break;
    }

    return 0;
}

/*
int retCode = 0;
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        return 1;
    }


    SDL_Window* window = SDL_CreateWindow("TGX Console", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 640, 480, SDL_WINDOW_SHOWN);

    if (window == NULL) {
        retCode = 1;
        printf("Error creating SDL window\n");
    goto SDL_END;
    }
    printf("Window created successfully\n");

    SDL_ShowWindow(window);
    bool running = true;

    printf("Entering main loop: \n");
    while (running) {
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            switch (ev.type) {
                case SDL_QUIT:
                    running = false;
                    break;
                default:
                    break;
            }
        }
    }

SDL_END:
    SDL_Quit();
    return retCode;
 **/
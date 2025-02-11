#include "TGX.h"


int load_rom(TGXContext* context, size_t *length);

uint32_t swi_handler(void* context, uint32_t code);

static bool debug_trace_enabled = false;

int execute(TGXContext *context) {
    if (alloc_principle_memory(&context->Memory) != TGX_SUCCESS) {
        printf("Unable to allocate required memory to start the vm. \n");
        return 1;
    }

    init_program_thread(&context->PU);
    init_graphics_thread(&context->GU, &context->Memory);

    if (load_rom(context, NULL) != TGX_SUCCESS) {
        return 1;
    }

    context->SWIFunc = swi_handler;

    program_thread_exec(context);
    return 0;
}

int main(int argc, char** argv) {
    TGXContext context = {0};

    if (argc > 1 && strcmp(argv[1], "--dis") == 0) {
        if (alloc_principle_memory(&context.Memory) != TGX_SUCCESS) {
            printf("Unable to allocate required memory to start the vm. \n");
            return 1;
        }

        size_t length;
        if (load_rom(&context, &length) != TGX_SUCCESS) {
            printf("Error loading rom\n");
            return 1;
        }

        Instruction* inst = (Instruction*)context.Memory.rom_begin;
        Instruction* e = (Instruction*)(context.Memory.rom_begin + length);

        FILE *f = fopen("./dis.asm", "w");

        if (f == NULL) {
            printf("Error opening file for writing, defaulting to stdout.\n");
            f = stdout;
        }

        while (inst < e) {
            fPrintInst(f, inst);
            inst++;
        }
        fclose(f);
        return 0;
    }

    if (argc > 1 && strcmp(argv[1], "--trace") == 0) {
        context.debug_trace = fopen("debug_trace_log.txt", "w");
    }

    do {
        if (execute(&context) != 0) {
            break;
        }

        printf("Signaling Graphics Shutdown...");

        context.GU.shutdown_flag = true;
        TriggerGraphics(&context.GU);
        SDL_WaitThread(context.GU.thread, NULL);

        printf("done.\n");

        SDL_DestroyMutex(context.GU.mutex);
        SDL_DestroyCond(context.GU.cond);

        printf("Shutting down\n");

        if (context.ExitCode == TGX_EXIT_CODE_RESTART) {
            free_principle_memory(&context.Memory);
            destroy_graphics_thread(&context.GU);
            destroy_program_thread(&context.PU);

            /*context.GU.shutdown_flag = false;
            context.ExitCode = TGX_EXIT_CODE_NONE;
            */

            context = (TGXContext){0};

            printf("Restarting Machine\n");
            continue;
        }
        break;
    } while (true);

    if (context.debug_trace != NULL) {
        fclose(context.debug_trace);
    }

    return 0;
}
// Things officially work. In that the baseline instruction set is good and needs to be tested.
// Next step is make an assembler and test the instructions

int load_rom(TGXContext* ctx, size_t *size) {
    FILE *f = fopen("./root/boot/krom", "rb");
    if (!f) {
        perror("Failed to open kernel rom file");
        return TGX_ERROR;
    }

    fseek(f, 0, SEEK_END);
    uint64_t file_size = ftell(f);
    rewind(f);

    unsigned char *buffer = malloc(file_size);
    if (!buffer) {
        perror("Could not allocate rom buffer");
        fclose(f);
        return TGX_ERROR;
    }

    size_t bytes_read = fread(buffer, 1, file_size, f);
    if (bytes_read != file_size) {
        perror("File read error");
        free(buffer);
        fclose(f);
        return TGX_ERROR;
    }

    if (bytes_read > ROM_SIZE) {
        perror("Kernel rom file is too large.");
        free(buffer);
        fclose(f);
        return TGX_ERROR;
    }

    memcpy(ctx->Memory.rom_begin, buffer, bytes_read);

    if (size != NULL) {
        *size = file_size;
    }

    free(buffer);
    fclose(f);
    return TGX_SUCCESS;
}

uint32_t swi_handler(void* context, uint32_t code) {
    TGXContext* ctx = context;

    switch (code) {
        case 0x0000A000:
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            return (uint32_t)(ts.tv_sec * 1000LL + ts.tv_nsec / 1000000LL);
        case 0xF0000000:
            for (int i=0; i< REG32_COUNT; i++) {
                printf("%2X: %8X (%u)\n", i, ctx->PU.gp32[i].full, ctx->PU.gp32[i].full);
            }
            break;
        case 0xF0000001:
            const char* str = (const char*)(ctx->Memory.memory_begin + ctx->PU.gp32[0].full);
            while (*str) {
                char ch = *str;
                if (ch == '%') {
                    str++;
                    if (!str) {
                        putc(ch, stdout);
                        break;
                    }
                    switch (*str) {
                        case 'd': {
                            int32_t val = *(int32_t*)(ctx->Memory.memory_begin + ctx->PU.gp32[REG_SP].full);
                            ctx->PU.gp32[REG_SP].full += sizeof(int32_t);
                            printf("%d", val);
                            break;
                        }
                        case 'u': {
                            uint32_t val = *(uint32_t*)(ctx->Memory.memory_begin + ctx->PU.gp32[REG_SP].full);
                            ctx->PU.gp32[REG_SP].full += sizeof(uint32_t);
                            printf("%u", val);
                            break;
                        }
                        case 'x': {
                            uint32_t val = *(uint32_t*)(ctx->Memory.memory_begin + ctx->PU.gp32[REG_SP].full);
                            ctx->PU.gp32[REG_SP].full += sizeof(uint32_t);
                            printf("%8X", val);
                            break;
                        }
                        case 'f': {
                            float val = *(float*)(ctx->Memory.memory_begin + ctx->PU.gp32[REG_SP].full);
                            ctx->PU.gp32[REG_SP].full += sizeof(float);
                            printf("%f", val);
                            break;
                        }
                        case 's': {
                            const char* str = (const char*)(ctx->Memory.memory_begin + ctx->PU.gp32[REG_SP].full);
                            ctx->PU.gp32[REG_SP].full += sizeof(uint32_t);
                            printf("%s", str);
                            break;
                        }
                        case 'l': {
                            int64_t val = *(int64_t*)(ctx->Memory.memory_begin + ctx->PU.gp32[REG_SP].full);
                            ctx->PU.gp32[REG_SP].full += sizeof(int64_t);
                            printf("%ld", val);
                            break;
                        }
                        case 'w': {
                            uint64_t val = *(uint64_t*)(ctx->Memory.memory_begin + ctx->PU.gp32[REG_SP].full);
                            ctx->PU.gp32[REG_SP].full += sizeof(uint64_t);
                            printf("%lu", val);
                            break;
                        }
                        case 'z': {
                            uint64_t val = *(uint64_t*)(ctx->Memory.memory_begin + ctx->PU.gp32[REG_SP].full);
                            ctx->PU.gp32[REG_SP].full += sizeof(uint64_t);
                            printf("%16lX", val);
                            break;
                        }
                        default:
                            putc(ch, stdout);
                            putc(*str, stdout);
                            break;
                    }
                    str++;
                    continue;
                }
                putc(ch, stdout);
                str++;
            }
            break;
        case 0xF0000002: {
            // scanf   RA::FmtPointer (expects a single entry) RB::MemoryPtr (expects it to be big enough for the entry)
            char buffer[256];
            char* fmt = (char*)(ctx->Memory.memory_begin + ctx->PU.gp32[REG_RA].full);
            void* dest = (ctx->Memory.memory_begin + ctx->PU.gp32[REG_RB].full);
            fgets(buffer, sizeof(buffer), stdin);
            sscanf(buffer, fmt, dest);
            break;
        }
        case 0xF0000003: {
            // fgets RA:BufferPointer RB::BufferSize
            char* buffer = (char*)(ctx->Memory.memory_begin + ctx->PU.gp32[REG_RA].full);
            uint32_t size = ctx->PU.gp32[REG_RB].full;
            fgets(buffer, size, stdin);
            break;
        }
        case 0xF0000004: {
            // scanf --> stores result directly into RA
            scanf("%d", &ctx->PU.gp32[REG_RA].full);
            break;
        }
        case 0xF0000005: {
            // getchar --> RA
            ctx->PU.gp32[REG_RA].full = getchar();
            break;
        }
        case 0xF0000006: {
            // putc (RA)
            putc(ctx->PU.gp32[REG_RA].full, stdout);
            break;
        }
        default:
            printf("Unknown syscode: 0x%08X (%u)\n", code, code);
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
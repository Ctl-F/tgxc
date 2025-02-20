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
        Instruction* start = inst;
        Instruction* e = (Instruction*)(context.Memory.rom_begin + length);

        FILE *f = fopen("./dis.asm", "w");

        if (f == NULL) {
            printf("Error opening file for writing, defaulting to stdout.\n");
            f = stdout;
        }

        while (inst < e) {
            fprintf(f, "%016lX: ", (size_t)(inst - start));
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

        printf("Shutting down\n");

        context.GU.shutdown_flag = true;
        TriggerGraphics(&context.GU);
        SDL_WaitThread(context.GU.thread, NULL);

        SDL_DestroySemaphore(context.GU.cpu_signal_gpu);
        SDL_DestroySemaphore(context.GU.gpu_signal_finished);

        if (context.ExitCode == TGX_EXIT_CODE_RESTART) {
            free_principle_memory(&context.Memory);
            destroy_graphics_thread(&context.GU);
            destroy_program_thread(&context.PU);

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

#define POP(type, context) *(type*)(context->Memory.memory_begin + context->PU.gp32[REG_SP].full); context->PU.gp32[REG_SP].full += sizeof(type)

typedef struct {
    struct {
        uint32_t mode;
        void* result_buffer;
        void* mapping_buffer;
    } DirectInput;
    struct {
        uint32_t mode;
        void* result_buffer;
    } HostInput;
    void* memory_begin;
} InputContext;

static InputContext s_InputController = (InputContext){0};
static char* swi_ErrorBuffer = NULL;

static void swi_Error(uint32_t code, const char* message) {
    if (swi_ErrorBuffer == NULL) return;

    *(uint32_t*)swi_ErrorBuffer = code;
    memccpy(swi_ErrorBuffer + sizeof(uint32_t), message, '\0', 1024);
}

void HandleKeyInput(SDL_Event* event) {

    switch (s_InputController.DirectInput.mode) {
        default:
        case 0: return;

        case 1: // controller input
        {
            // TODO:
            return;
        }
        case 2: // Keyboard Mapped Input
        {
            uint32_t *keyboard_map = (uint32_t*)s_InputController.DirectInput.mapping_buffer;
            uint8_t *result_buffer = (uint8_t*)s_InputController.DirectInput.result_buffer;

            for (int i=0; i<10; i++) {
                if (event->key.keysym.scancode == keyboard_map[i]) {
                    result_buffer[i] = event->key.state;
                    break;
                }
            }
            return;
        }
        case 3: // Keyboard Full Input
        {
            if (event->key.keysym.scancode >= INPUT_SCAN_CODE_END) return;

            uint32_t p_cFrame;
            p_cFrame = *(uint32_t*)(s_InputController.DirectInput.result_buffer + 16);
            uint8_t* currentFrame = s_InputController.memory_begin + p_cFrame;
            currentFrame[event->key.keysym.scancode] = event->key.state;
            return;
        }
    }

}

void HandleMouseButtonInput(SDL_Event* event) {
    switch (s_InputController.DirectInput.mode) {
        default:
        case 0:
        case 1: return;
        case 2: // keyboard mapped input
        {
            uint8_t* btn_buffer = (uint8_t*)(s_InputController.DirectInput.result_buffer + 11 * sizeof(uint32_t));

            if (event->button.button == SDL_BUTTON_LEFT) {
                *btn_buffer = event->button.state;
            }
            else {
                *(btn_buffer+1) = event->button.state;
            }
            return;
        }
        case 3: {
            uint8_t* buffer = (uint8_t*)(s_InputController.DirectInput.result_buffer + 8);

            if (event->button.button > 3) return;

            buffer[event->button.button - 1] = event->button.state;
            return;
        }
    }
}

void HandleMouseMovementInput(SDL_Event* event) {
    switch (s_InputController.DirectInput.mode) {
        default:
        case 0:
        case 1: return;
        case 2: // keyboard mapped input
        {
            float* btn_buffer = (float*)(s_InputController.DirectInput.result_buffer + 11 * sizeof(uint32_t) + 2);

            *btn_buffer = (float)event->motion.xrel;
            *(btn_buffer + 1) = (float)event->motion.yrel;
            return;
        }
        case 3: {
            float* buffer = (float*)s_InputController.DirectInput.result_buffer;
            *buffer = (float)event->motion.x;
            *(buffer+1) = (float)event->motion.y;
            return;
        }
    }
}

void HandleMouseWheelInput(SDL_Event* event) {
    if (s_InputController.DirectInput.mode != 3) {
        return;
    }

    int32_t* buffer = (int32_t*)(s_InputController.DirectInput.result_buffer + 11);
    *buffer = event->wheel.y;
}

void HandleTextInput(SDL_Event* event) {

}

uint32_t swi_handler(void* context, uint32_t code) {
    TGXContext* ctx = context;

    switch (code) {
        default: {
            swi_Error(SWI_ErrorInvalidSysCode, "Unknown syscall code.");
            return 1;
        }
        case 0x00000010: // InputControllerSetMode
        {
            uint32_t mode = POP(uint32_t, ctx);
            s_InputController.DirectInput.mode = mode;
            if (mode == 0) {
                return 1;
            }

            if (mode > 3) {
                swi_Error(SWI_ErrorInvalidInputMode, "Invalid input mode provided for input controller. Allowed values are 0: Disabled, 1: Controller, 2: KeyboardMapped, 3: KeyboardFull.");
                return 0;
            }

            uint32_t pDataBuffer = POP(uint32_t, ctx);
            s_InputController.DirectInput.result_buffer = ctx->Memory.memory_begin + pDataBuffer;
            s_InputController.DirectInput.mapping_buffer = NULL;
            if (mode == 2) {
                uint32_t pMappingBuffer = POP(uint32_t, ctx);
                s_InputController.DirectInput.mapping_buffer = ctx->Memory.memory_begin + pMappingBuffer;
            }
            return 1;
        }
        case 0x00000011: // InputControllerSetHostMode
        {
            uint32_t mode = POP(uint32_t, ctx);
            s_InputController.HostInput.mode = mode;

            if (mode == 0) {
                return 1;
            }

            s_InputController.memory_begin = ctx->Memory.memory_begin;

            if (mode != 1) {
                swi_Error(SWI_ErrorInvalidInputHostMode, "Invalid host input mode provided for input controller. Allowed values are 0: Disabled, 1: Enabled");
                return 0;
            }
            uint32_t pBuffer = POP(uint32_t, ctx);
            s_InputController.HostInput.result_buffer = ctx->Memory.memory_begin + pBuffer;
            return 1;
        }
        case 0x00000012: {
            if (s_InputController.DirectInput.mode == 3) {
                uint32_t p_cFrame, p_lFrame;
                p_cFrame = *(uint32_t*)(s_InputController.DirectInput.result_buffer + 16);
                p_lFrame = *(uint32_t*)(s_InputController.DirectInput.result_buffer + 20);

                uint8_t* currentFrame = s_InputController.memory_begin + p_cFrame;
                uint8_t* lastFrame = s_InputController.memory_begin + p_lFrame;

                memcpy(lastFrame, currentFrame, INPUT_SCAN_CODE_END);
            }


            SDL_Event event;
            while (SDL_PollEvent(&event)) {
                switch (event.type) {
                    case SDL_QUIT:
                        if (s_InputController.HostInput.mode) {
                            *(uint8_t*)s_InputController.HostInput.result_buffer = 1;
                        }
                        break;
                    case SDL_KEYUP:
                    case SDL_KEYDOWN:
                        HandleKeyInput(&event);
                        break;
                    case SDL_MOUSEBUTTONUP:
                    case SDL_MOUSEBUTTONDOWN:
                        HandleMouseButtonInput(&event);
                        break;
                    case SDL_MOUSEMOTION:
                        HandleMouseMovementInput(&event);
                        break;
                    case SDL_MOUSEWHEEL:
                        HandleMouseWheelInput(&event);
                        break;
                    case SDL_TEXTINPUT:
                        HandleTextInput(&event);
                        break;
                    default: break;
                }
            }

            return 0;
        }
        case 0x00001000: {
            // Enable SWI Error Feedback.
            uint32_t pErrorBuffer = POP(uint32_t, ctx);
            if (pErrorBuffer == 0) {
                swi_ErrorBuffer = NULL;
                return 0;
            }

            swi_ErrorBuffer = (char*)(ctx->Memory.memory_begin + pErrorBuffer);
            return 1;
        }
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
                            uint32_t vPtr = *(uint32_t*)(ctx->Memory.memory_begin + ctx->PU.gp32[REG_SP].full);
                            const char* _str = (const char*)(ctx->Memory.memory_begin + vPtr);
                            ctx->PU.gp32[REG_SP].full += sizeof(uint32_t);
                            printf("%s", _str);
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
            return ctx->PU.gp32[REG_RA].full;
        }
        case 0xF0000005: {
            // getchar --> RA
            return ctx->PU.gp32[REG_RA].full = getchar();
        }
        case 0xF0000006: {
            // putc (RA)
            putc(ctx->PU.gp32[REG_RA].full, stdout);
            break;
        }
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
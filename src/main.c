#include "TGX.h"

/*
typedef struct {
    union {
        uint8_t hb[4];
        uint16_t hl[2];
        uint32_t f;
    };
} foo;
*/

int main(void) {
    int retCode = 0;
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        return 1;
    }

    /*
    foo bar;
    bar.f = 0xDEADBEEF;

    printf("%x\n%x   %x\n%x  %x  %x  %x\n", bar.f, bar.hl[0], bar.hl[1], bar.hb[0], bar.hb[1], bar.hb[2], bar.hb[3]);
    */

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
}

#ifndef AI_WRAPPER_H
#define AI_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    int category;
    float score;
    int x1;
    int y1;
    int x2;
    int y2;
} detection_t;

void ai_init(void);
//void ai_run(uint8_t *image_buf, int width, int height);
//detection_t ai_run(uint8_t *image_buf, int width, int height);
void ai_run(uint8_t *image_buf, int width, int height, detection_t *output);
#ifdef __cplusplus
}
#endif

#endif

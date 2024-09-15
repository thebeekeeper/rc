#ifndef JS_DATA_H
#define JS_DATA_H

typedef struct js_reading_S {
    int8_t left_horizontal;
    int8_t left_vertical;
    int8_t right_horizontal;
    int8_t right_vertical;
    bool left_button;
    bool right_button;
} js_reading;

#endif
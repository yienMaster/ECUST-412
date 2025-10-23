#ifndef JUMP_CONTROL_H
#define JUMP_CONTROL_H


// ÌøÔ¾×´Ì¬»ú
typedef enum {
    JUMP_PREPARE,       // ×¼±¸½×¶Î
    JUMP_BURST,         // ±¬·¢½×¶Î
    JUMP_AIR_RETRACT,   // ¿ÕÖÐÊÕÍÈ
    JUMP_PRE_BUFFER,    // ×¼±¸»º³å
    JUMP_EXE_BUFFER,    // Ö´ÐÐ»º³å
    JUMP_RECOVER  ,     // »Ö¸´½×¶Î
    JUMP_FREE//¿ÕÏÐ×´Ì¬
} JumpState;
extern int jump_stop;
extern JumpState jump_state;
void jump_process_control(float *current_leg_length,float *angle);
void jump_abort(void);

#endif

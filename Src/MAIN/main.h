#ifndef MAIN_H
#define MAIN_H

#include "c_defs.h"
#include "compiler_defs.h"


typedef enum
{
	NORMAL_MODE = 0u,
	DEBUG_MODE
} MODE_type_et;


void delay_ms(u16_t);
void delay_us(u16_t us);
u8_t generate_random_number( void );

void         set_operating_mode( MODE_type_et mode );
MODE_type_et get_operating_mode( void );


#endif

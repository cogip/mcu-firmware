#pragma once

/* Small terminal helpers shared by the motor/QDEC bench test. */

void clear_screen(void);
void hr(void);
void wait_key(const char *prompt);
int  ask_char(const char *prompt, const char *valid);

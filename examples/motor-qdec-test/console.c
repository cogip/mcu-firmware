#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "console.h"

void clear_screen(void)
{
    printf("\033[2J\033[H");
    fflush(stdout);
}

void hr(void)
{
    puts("------------------------------------------------------------");
}

void wait_key(const char *prompt)
{
    printf("%s", prompt);
    fflush(stdout);
    getchar();
}

int ask_char(const char *prompt, const char *valid)
{
    for (;;) {
        printf("%s", prompt);
        fflush(stdout);

        int c;
        do {
            c = getchar();
        } while (c == '\n' || c == '\r' || c == ' ' || c == '\t');

        c = tolower(c);
        if (strchr(valid, c) != NULL) {
            printf("%c\n", c);
            return c;
        }
        puts("  ? invalid, try again");
    }
}

#include <stdio.h>
int main(int argc, char *argv[])
{
    printf("arguments are:\n");
    for (int i = 0; i < argc; i++)
        printf("    %d: %lx %s\n", i, argv[i], argv[i]);
    return 0;
}
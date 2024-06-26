#include <stdio.h>
int main()
{
    long a, b;
    printf("Input a b: ");
    fflush(stdout);
    scanf("%ld%ld", &a, &b);
    printf("%ld\n", a + b);
    return 0;
}
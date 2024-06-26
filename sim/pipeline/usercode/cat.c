#include <stdio.h>
#include <stdlib.h>
int main()
{
    char s[128];
    printf("please input filename: ");
    fflush(stdout);
    scanf("%s", s);
    FILE *fp = fopen(s, "r");
    if (!fp)
        return printf("unable to open file %s\n", s), 117;
    fseek(fp, 0, SEEK_END);
    int sz = ftell(fp);
    rewind(fp);
    char *buf = (char *)malloc(sz);
    if (!buf)
        return printf("malloc fails\n"), 83;
    fread(buf, sz, 1, fp);
    fwrite(buf, sz, 1, stdout);
    return 0;
}
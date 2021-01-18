#include <stdio.h>
#include <stdlib.h>

void bin(unsigned n)
{
    unsigned i;
    for (i = 1 << 31; i > 0; i = i / 2)
        (n & i) ? printf("1") : printf("0");
}

int main()
{
    int num, flippedNumber;

    /* Input number from user */
    num = 0b00111111;

    flippedNumber = ~num;

    char buffer [33];
    snprintf(buffer, sizeof(buffer), "%d", flippedNumber);
    printf ("decimal: %s\n",buffer);
    printf ("binary: %p\n",buffer);
    printf("-----\n");
    bin(num);
    printf("\n-----\n");
    bin(flippedNumber);
    printf("\n-----\n");

    printf("Original number = %d (in decimal)\n", num);
    printf("Number after bits are flipped = %hhX (in decimal)", flippedNumber);

    return 0;
}

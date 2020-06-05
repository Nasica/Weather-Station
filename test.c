#include "readTemp.h"
#include <stdio.h>

int main() {
    int fd; 
    fd = initGPIO(0, 0x76);
    int temp = getTemp(fd);
    printf("%d\n",temp);
    return(0);
}



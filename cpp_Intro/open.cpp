#include <fcntl.h>
#include <stdio.h>

namespace robolab
{
    int open(const char *filename, int flags)
    {
        printf("Die eigene Funktion open wurde aufgerufen\n");
        return 0;
    }
}

int main(int argc, char *argv[])
{
    const char filename[] = "file.txt";
    int retval = robolab::open(filename, 0);
    return 0;
}

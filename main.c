#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <string.h>
#include <fcntl.h>  // used for non-blocking pipes
#define READ_BLOCK_SIZE 180
#define CHILD_SLEEP_TIME 1


int main(int argc, char *argv[3])
{

    int readFromStartupFd, readFromSimFd, readFromContFd;

    if(argc != 4) {
        printf("Display Spawn Error: Expecting 4 arguments, received %i\n", argc);
        exit(1);
    }

    printf("Display: Overlay successful\n\n");

    readFromStartupFd = atoi(argv[1]);
    readFromSimFd = atoi(argv[2]);
    readFromContFd = atoi(argv[3]);
    printf("Display: Startup to Display Pipe read fd from command line argument is %i\n", readFromStartupFd);
    printf("Display: Simulator to Display Pipe read fd from command line argument is %i\n", readFromSimFd);
    printf("Display: Controller to Display Pipe read fd from command line argument is %i\n", readFromContFd);
    printf("\n");

        /* ***************** Set read end to non-blocking mode on display **************************/


    printf("Display: Setting read end of Simulator to Startup process pipe to non-blocking mode\n");

    if (fcntl(readFromSimFd, F_SETFL, O_NONBLOCK) < 0) {
        perror("non-blocking");
        exit(2);
    }

    printf("Display: Setting read end of Controller to Startup process pipe to non-blocking mode\n");

    if (fcntl(readFromContFd, F_SETFL, O_NONBLOCK) < 0) {
        perror("non-blocking");
        exit(2);
    }

    printf("Display: Setting read end of Startup to Display process pipe to non-blocking mode\n");

    if (fcntl(readFromStartupFd, F_SETFL, O_NONBLOCK) < 0) {
         perror("non-blocking");
         exit(2);
    }
    printf("\n");
    /* ********************************************************************************************/

         printf("Display: Ready to read pipe messages from Startup/Controller/Simulator\n");
         char readBufferDisplay[READ_BLOCK_SIZE+1];
         ssize_t bytesReadDisplay;
         char readBufferSim[READ_BLOCK_SIZE+1];
         ssize_t bytesReadSim;
         char readBufferCont[READ_BLOCK_SIZE+1];
         ssize_t bytesReadCont;

    while(1){

            /* ***********  Section for reading incoming pipe messages from Startup    ************/

            bytesReadDisplay = read(readFromStartupFd, readBufferDisplay, READ_BLOCK_SIZE);
            // if bytesReadSim is positive, bytes have been read from the pipe

            while (bytesReadDisplay > 0) {
                // terminate string in readBufferSim for printing
                readBufferDisplay[bytesReadDisplay] = '\0';
                //printf("Display: Read %li bytes from Startup - '%s'\n", bytesReadDisplay, readBufferDisplay);
                printf("'%s'\n", readBufferDisplay);
                bytesReadDisplay = read(readFromStartupFd, readBufferDisplay, READ_BLOCK_SIZE);
            }

            /* ***********  Section for reading incoming pipe messages from Simulator    ************/

            bytesReadSim = read(readFromSimFd, readBufferSim, READ_BLOCK_SIZE);

            // if bytesReadSim is positive, bytes have been read from the pipe
            while (bytesReadSim > 0) {
                // terminate string in readBufferSim for printing
                readBufferSim[bytesReadSim] = '\0';
               // printf("Display: Read %li bytes from Simulator - '%s'\n", bytesReadSim, readBufferSim);
                printf("Sim: '%s'\n",readBufferSim);
                bytesReadSim = read(readFromSimFd, readBufferSim, READ_BLOCK_SIZE);
            }

            /* ***********  Section for reading incoming pipe messages from controller    ***********/

             bytesReadCont = read(readFromContFd, readBufferCont, READ_BLOCK_SIZE);

            // if bytesReadSim is positive, bytes have been read from the pipe
            while (bytesReadCont > 0) {
                // terminate string in readBufferSim for printing
                readBufferCont[bytesReadCont] = '\0';
                printf("Contr: '%s'\n", readBufferCont);
                bytesReadCont = read(readFromContFd, readBufferCont, READ_BLOCK_SIZE);
            }

    }

}

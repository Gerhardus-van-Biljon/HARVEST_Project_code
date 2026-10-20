//for the 3rd time and certainly not the last the includes that are being used
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pid.h"

//This is now calling that of the previous kinematics functions to run and bring them together in sweet harmony
void run_forward_kinematics(void);
void run_inverse_kinematics(void);

//aain for the 3rd time dont worry here just allows me to type in the values as explained in the previous two
static void get_choice(char *buffer, size_t max_len)
{
    size_t idx = 0;
    while (idx < max_len - 1) {
        int c = getchar();
        if (c == EOF || c == 255) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (c == '\r' || c == '\n') {
            putchar('\n');
            break;
        }
        putchar(c);
        fflush(stdout);
        buffer[idx++] = (char)c;
    }
    buffer[idx] = '\0';
}
//this is the main function that deploys all the files it be the main one 
void app_main(void)
{
    //starts the PID controller in the background which we will get to momentarilly
    start_pid_task();

    char buf[16];
    //all this does is run the menu and ask to either press that of 1 for FK and 2 for IK there is not much to really talk about here so
    //yeah...... i am pretty sure you understand this and will rather move onto explaining the PID controller stuff so lets go there in its file
    while (1) {
        printf("\n====================================\n");
        printf("       Kinematics Main Menu         \n");
        printf("====================================\n");
        printf("1. Run Forward Kinematics (FK)\n");
        printf("2. Run Inverse Kinematics (IK)\n");
        printf("Select option (1 or 2): ");
        fflush(stdout);

        get_choice(buf, sizeof(buf));

        if (buf[0] == '1') {
            run_forward_kinematics();
        } else if (buf[0] == '2') {
            run_inverse_kinematics();
        } else {
            printf("Invalid selection. Enter 1 or 2.\n");
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
//The includes once aagin and the defined varibles
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pid.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define RAD_TO_DEG(rad) ((rad) * (180.0f / M_PI))
#define DEG_TO_RAD(deg) ((deg) * (M_PI / 180.0f))

static float wrap_angle(float a) {
    return atan2f(sinf(a), cosf(a));
}

// heres all the structures that were created 
//The first one being none other than that of the target position
struct TargetPosition {
    float x;
    float y;
    float z;
    float yaw;
};
//then we get target orientation
struct TargetOrientation {
    float roll;
    float pitch;
    float yaw;
};
//then lets not forget the infamous joint angles wrapping it all up
struct JointAngles {
    float q_rail;
    float q1;
    float q_shoulder; // corresponds to q2
    float q_elbow;    // corresponds to q3
    float q4;
    float q5;
    float q6;
};

//i will just copy and paste what i said previously about this:
//copy
//okay this function is nothing to worry about its whole job is just allowing to type in values for forward kinematics
//as a previous issues caused it to keep printing without waiting for me to type in my answer all this does is let wait for me to press 
//enter before continuing. so dont worry about this at all
//Pasted
static void get_line(char *buffer, size_t max_len) {
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

//here is now the main function of this file which shall be called in the true main file 
//so lets get into the explaination
void run_inverse_kinematics(void) {
    //first off it calls the structures and gives them shorthand names as well as defining other varibles
    struct TargetPosition target;
    struct JointAngles angles;
    char input_buffer[128];
    
    //Link dimensions matching the analytical node model
    float d1 = 0.50f, d2 = 0.50f, d3 = 0.50f;
    float a4 = 0.50f, a5 = 0.50f, a6 = 0.50f;
    float wrist_roll_val = 0.0f;
    int elbow_branch = 0;
    float yaw_deg = 0.0f;

    //here is where it starts printing the mode it in being that of IK mode when selected
    printf("\n====================================\n");
    printf("      Inverse Kinematics Mode       \n");
    printf("====================================\n");
    //This is the while loop like that of the forward kinematics
    while (1) {
        //keeps asking for the position until you enter them or press q to exit that of the main menu which will be shown in the main file trust
        printf("\nEnter target X Y Z Yaw_deg (or 'q' to main menu):\n> ");
        fflush(stdout);
        //just the buffer to allow me to type is all
        get_line(input_buffer, sizeof(input_buffer));
        //line that allows me to break once i press q
        if (input_buffer[0] == 'q' || input_buffer[0] == 'Q') {
            printf("Returning to Main Menu...\n");
            break;
        }
        //where the values are stored once you type in the pose of the robot
        if (sscanf(input_buffer, "%f %f %f %f",
                   &target.x,
                   &target.y,
                   &target.z,
                   &yaw_deg) == 4) {
            
            float phi = DEG_TO_RAD(yaw_deg);

            // 3. Rail (vertical): z = d1 + q_rail + d2
            angles.q_rail = target.z - (d1 + d2);
            angles.q_shoulder = 0.0f; // shoulder locked for planar operation
            angles.q5 = wrist_roll_val;

            // 4. Wrist point: back off from the target along links 6 and 5
            float Xw = target.x - a6 * cosf(phi) - a5 * cosf(phi - angles.q5);
            float Yw = target.y - a6 * sinf(phi) - a5 * sinf(phi - angles.q5);

            // 5. Elbow calculation
            float sin_q_elbow = (Xw * Xw + Yw * Yw - d3 * d3 - a4 * a4) / (2.0f * d3 * a4);
            if (sin_q_elbow < -1.0f || sin_q_elbow > 1.0f) {
                printf("Target is out of reach.\n");
                continue;
            }
            angles.q_elbow = (elbow_branch == 0)
                                 ? asinf(sin_q_elbow)
                                 : wrap_angle((float)M_PI - asinf(sin_q_elbow));

            // 6. Base calculation
            float u = a4 * cosf(angles.q_elbow);
            float v = d3 + a4 * sinf(angles.q_elbow);
            if (u * u + v * v < 1e-6f) {
                printf("Singular configuration, no solution.\n");
                continue;
            }
            angles.q1 = atan2f(u * Yw - v * Xw, u * Xw + v * Yw);

            // 7. Wrist pitch absorbs remaining yaw
            angles.q4 = wrap_angle(phi - angles.q1 - angles.q_elbow - angles.q5);
            angles.q6 = angles.q5;
            
            //this takes the angles in degress and changes them to radians to send to the PID controller update
            // Note: mapping 6 active joint targets (Rail mapped or assigned to index 0 if appropriate)
            g_target_angles[0] = RAD_TO_DEG(angles.q1);
            g_target_angles[1] = RAD_TO_DEG(angles.q_shoulder);
            g_target_angles[2] = RAD_TO_DEG(angles.q_elbow);
            g_target_angles[3] = RAD_TO_DEG(angles.q4);
            g_target_angles[4] = RAD_TO_DEG(angles.q5);
            g_target_angles[5] = RAD_TO_DEG(angles.q6);

            //prints the target angles at the very end
            printf("\nComputed Joint Angles (degrees):\n");
            printf("Rail Joint   : %8.2f m\n", angles.q_rail);
            printf("q1 (Base)    : %8.2f deg\n", g_target_angles[0]);
            printf("q2 (Shoulder): %8.2f deg\n", g_target_angles[1]);
            printf("q3 (Elbow)   : %8.2f deg\n", g_target_angles[2]);
            printf("q4 (Wrist P) : %8.2f deg\n", g_target_angles[3]);
            printf("q5 (Wrist R) : %8.2f deg\n", g_target_angles[4]);
            printf("Targets updated for PID controller!\n");
        } else {
            printf("Invalid format. Enter X Y Z Yaw or 'q'.\n");
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
//so this is the end of the IK and i hope it all made sense and if not contact local ginger friend named Liam for any issues and all help
//or if you want to correct him and tell him he made mistakes which he definitely doesnt do on the regular and prefers to call them rather
//just not being 100% correct right now errors
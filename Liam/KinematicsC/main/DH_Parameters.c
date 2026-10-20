//These are the included libaries or at least I think they are still called libaries 
//whatever they are the includes and the defined specifics
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define NUM_JOINTS 6
#define DEG2RAD(deg) ((deg) * M_PI / 180.0)
#define RAD2DEG(rad) ((rad) * 180.0 / M_PI)

// Link dimensions matching the node definitions
static double d1 = 0.50;
static double d2 = 0.50;
static double d3 = 0.50;
static double a4 = 0.50;
static double a5 = 0.50;
static double a6 = 0.50;

//This is the structure that helps define the parameters your theta, d, a and alpha 
//I do apologise for my terrible comments in advance 
//anywho moving on the structs name is DH_Parameter which will be used later in the near future 
typedef struct {
    double theta;
    double d;
    double a;
    double alpha;
} DH_Parameter; // here is the structs name thought i would say so

//And as promised the near future is hear and we are using the DH_Parameter struct
//So we create a static DH table matching the node DH configuration (Rail, Base, Shoulder, Elbow, Wrist Pitch, Wrist Roll)
static DH_Parameter robotArm[NUM_JOINTS] = {
    {0.0,           0.0, 0.0,   0.0}, // Joint 0: rail_joint (d updated dynamically with q_rail)
    {0.0,           0.0, 0.0, -90.0}, // Joint 1: base_joint
    {0.0,           0.0, 0.0,  90.0}, // Joint 2: shoulder_joint
    {0.0,           0.0,  0.0,  0.0}, // Joint 3: elbow_joint
    {0.0,           0.0,  0.0,  0.0}, // Joint 4: wrist_pitch_joint
    {0.0,           0.0,  0.0, -90.0}  // Joint 5: wrist_roll_joint
};

//Here is our first function The multiply function yippie!!!
//well anyways the multiply functions as the name suggests whitch it takes the matrices
//the 4x4 Homogenous Transformation Matrices and multiplys them against one another until the result I will go into detail now how it works
//lets start from the top the values used in here are A,B and result all are doubles while being 4x4 matrices
static void multiply(const double A[4][4], const double B[4][4], double result[4][4]) {
    //So we create a temp matrix which cycles in a for loop
    double temp[4][4] = {0};
    for (int i = 0; i < 4; i++) //here is the for loop whitch is nested this is i
        for (int j = 0; j < 4; j++) // this is j
            for (int k = 0; k < 4; k++) // this is k 
                temp[i][j] += A[i][k] * B[k][j]; // okay here is cycles through and adds them 
    //here is another for loop which takes the values from the temp and equals them to the result
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            result[i][j] = temp[i][j];
}
//also a friendly reminder if you see any errors in the code please feel free to point them out and tell me and i will get to fixing it lickity
//split
//:)

//here is the second function which DH Matrix not to be confused with the Dh table and parameters
//it takes the strucs values being that of theta, d, a. Though it adds alphaDEG and T 
//the alphaDEG is taking the inputed value later and changing it to alpha later in radians 
//and double T not to be confused for triple T(dumb joke from younger cousins referencing Tung Tung Tung sahur)anyways
//is the transformation matrix
static void DHMatrix(double theta, double d, double a, double alphaDeg, double T[4][4]) {
    //here it changes from degrees to Radians 
    double alpha = DEG2RAD(alphaDeg);
    //Okay here is the transformation matrix but here i need yall to specifcally check if it looks correct though it should be 
    T[0][0] = cos(theta);  T[0][1] = -sin(theta)*cos(alpha); T[0][2] =  sin(theta)*sin(alpha); T[0][3] = a*cos(theta);
    T[1][0] = sin(theta);  T[1][1] =  cos(theta)*cos(alpha); T[1][2] = -cos(theta)*sin(alpha); T[1][3] = a*sin(theta);
    T[2][0] = 0;           T[2][1] =  sin(alpha);            T[2][2] =  cos(alpha);            T[2][3] = d;
    T[3][0] = 0;           T[3][1] =  0;                     T[3][2] =  0;                     T[3][3] = 1;
}

//here is the third function Called extractRPY which you will never guess .... oh you got the guess right well i was not expecting that 
//well yeah it extracts the Roll, Pitch and Yaw anyhow it uses the R matrix which is the rotation matrix and uses the 
//roll, pitch and yaw varibles 
static void extractRPY(const double R[4][4], double *roll, double *pitch, double *yaw) {
    //here we are getting the Pitch, roll, yaw in radians also make sure the formulas are correct 
    double pitchRad = atan2(-R[2][0], sqrt(R[0][0] * R[0][0] + R[1][0] * R[1][0]));
    double rollRad  = atan2(R[2][1], R[2][2]);
    double yawRad   = atan2(R[1][0], R[0][0]);
    //converts from radians to that of degress this will be used to display unless you prefer radians 
    *roll  = RAD2DEG(rollRad);
    *pitch = RAD2DEG(pitchRad);
    *yaw   = RAD2DEG(yawRad);
}

//Okay we have arrived to The fourth and arguebly the most important function THE COMPUTEFK which computes the forward kinematics
//it pulls from the dh parameters as well as the Dh table 
static void computeFK(const DH_Parameter dh[NUM_JOINTS], double q_rail) {
    // Update dynamic d parameters for rail and link offsets
    DH_Parameter currentArm[NUM_JOINTS];
    for(int i = 0; i < NUM_JOINTS; i++) {
        currentArm[i] = dh[i];
    }
    currentArm[0].d = d1 + q_rail;
    currentArm[1].d = d2;
    currentArm[2].d = d3;
    currentArm[3].a = a4;
    currentArm[4].a = a5;
    currentArm[5].a = a6;

    //here is the final matrix or rather it will be
    double T_Total[4][4] = {
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };
    //here it creates the for loop that multiplys the matrices together to eventually get the end matrix
    for (int i = 0; i < NUM_JOINTS; i++) {
        double T_Joint[4][4];
        DHMatrix(currentArm[i].theta, currentArm[i].d, currentArm[i].a, currentArm[i].alpha, T_Joint);
        multiply(T_Total, T_Joint, T_Total);
    }
    //this here gets the roll, pitch and yaw from the previous function being extractRPY
    double roll, pitch, yaw;
    extractRPY(T_Total, &roll, &pitch, &yaw);//as shown here

    //Okay here we are this part just shows everything i dont really think i need to go into too much detail 
    //but heres the short of it displays position anf orientation Boom done short and sweet like me 
    printf("\n================ End-Effector Pose ================\n");
    printf("Position    X: %8.4f   Y: %8.4f   Z: %8.4f\n",
           T_Total[0][3], T_Total[1][3], T_Total[2][3]);
    printf("Orientation Roll: %6.2f deg  Pitch: %6.2f deg  Yaw: %6.2f deg\n",
           roll, pitch, yaw);
    printf("---------------------------------------------------\n");
    printf("Rotation matrix:\n");
    for (int i = 0; i < 3; i++) {
        printf("  [%8.4f %8.4f %8.4f]\n", T_Total[i][0], T_Total[i][1], T_Total[i][2]);
    }
    printf("===================================================\n");
}

//okay this function is nothing to worry about its whole job is just allowing to type in values for forward kinematics
//as a previous issues caused it to keep printing without waiting for me to type in my answer all this does is let wait for me to press 
//enter before continuing. so dont worry about this at all
static void readLine(char *buffer, size_t maxLen) {
    size_t index = 0;
    while (index < maxLen - 1) {
        int c = getchar();
        if (c == EOF || c == 255) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (c == '\r' || c == '\n') {
            putchar('\n');
            break;
        }

        putchar(c);
        fflush(stdout);
        buffer[index++] = (char)c;
    }
    buffer[index] = '\0';
}

//here you can worry about and honestly stress about...... of course i am joking ha ha yeah not funny i know but this is what you would
// the main as it makes everthing run though it will send it over to the main file and it runs the forward kinematics i shall now 
//go into more detail
void run_forward_kinematics(void) {
    //just says the mode it is in
    printf("\n====================================\n");
    printf("      Forward Kinematics Mode       \n");
    printf("====================================\n");
    //this is a while that lets it keep repeating asking for the theta values until i press q which then we go back and can choose between 
    //forward or inverse kinematics
    while (1) {
        printf("\nEnter Rail and %d joint angles (Rail_m theta1..theta6) or 'q' to menu:\n> ",
               NUM_JOINTS);
        fflush(stdout);

        char lineBuf[128] = {0};
        readLine(lineBuf, sizeof(lineBuf));

        if (lineBuf[0] == 'q' || lineBuf[0] == 'Q') {
            printf("Returning to Main Menu...\n");
            break;
        }
        //okay so after all that we entered it takes the degree values we enter and sees if we entered that of 7 values and if not
        //asks us again to reenter until we enter 7 or press q
        double q_rail;
        double thetaDeg[NUM_JOINTS];
        int parsed = sscanf(lineBuf, "%lf %lf %lf %lf %lf %lf %lf",
                            &q_rail, &thetaDeg[0], &thetaDeg[1], &thetaDeg[2],
                            &thetaDeg[3], &thetaDeg[4], &thetaDeg[5]);

        if (parsed != NUM_JOINTS + 1) {
            printf("Expected 7 numbers (Rail + 6 joints), received %d. Try again.\n", parsed);
            continue;
        }
        //here it converts it from degrees to that of radians so it can do its calculations later for the theta values
        for (int i = 0; i < NUM_JOINTS; i++) {
            robotArm[i].theta = DEG2RAD(thetaDeg[i]);
        }

        computeFK(robotArm, q_rail);
    }
}
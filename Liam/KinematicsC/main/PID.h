//Going to keep this short and sweet all ths is for is connecting the IK and PID file so they can communicate in a shared file so there is no 
//inconsitency between the two and then have less or no errors on that front is all
#ifndef PID_H
#define PID_H

#define NUM_JOINTS 6


extern float g_target_angles[NUM_JOINTS];
//defines the PID parameters
typedef struct {
    float kp;
    float ki;
    float kd;
    float setpoint;
    float integral;
    float prev_error;
    float output_min;
    float output_max;
} PIDController;
//and starts the tasks it needs to do
void init_pids(void);
void start_pid_task(void);

#endif // PID_H
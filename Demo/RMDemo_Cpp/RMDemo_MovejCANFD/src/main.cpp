#define _CRT_SECURE_NO_WARNINGS
#include <cstdlib>
#include <stdio.h>
#include <time.h>
#include <string.h>
#define arm_dof_angle 6
#define MAX_POINTS 5000
#ifdef _WIN32  
// Windows-specific headers and definitions
#include <windows.h>
#include <sys/types.h>
#include <direct.h>  // For _mkdir on Windows
#define SLEEP_MS(ms) Sleep(ms)
#define SLEEP_S(s) Sleep((s) * 1000)
#define usleep(us) Sleep((us) / 1000)
#define mkdir(dir, mode) _mkdir(dir)
#else
// Linux-specific headers and definitions
#include <unistd.h>
#include <sys/stat.h>
#include <sys/time.h>
#define SLEEP_MS(ms) usleep((ms) * 1000)
#define SLEEP_S(s) sleep(s)
#endif
#ifndef DATA_FILE_PATH
#error "DATA_FILE_PATH is not defined"
#endif

#include "rm_service.h"
RM_Service robotic_arm;

// Global variables for data saving
FILE* data_output_file = NULL;
float current_plan_point[arm_dof_angle] = {0};
int current_point_index = -1;
bool is_recording = false;

// Data recording switch - set to 1 to enable recording, 0 to disable
int data_recording_enabled = 1;

// Function to create data directory if it doesn't exist
void ensure_data_directory_exists() {
    #ifdef _WIN32
    mkdir("data", 0);
    #else
    mkdir("data", 0755);
    #endif
}

// Function to open data file with timestamp
void start_data_recording() {
    // Check if recording is enabled
    if (data_recording_enabled == 0) {
        printf("Data recording is disabled (switch is off)\n");
        return;
    }
    
    ensure_data_directory_exists();
    
    // Create filename with timestamp
    time_t now = time(NULL);
    struct tm* t = localtime(&now);
    char filename[100];
    sprintf(filename, "data/joint_data_%04d%02d%02d_%02d%02d%02d.txt", 
            t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
            t->tm_hour, t->tm_min, t->tm_sec);
    
    data_output_file = fopen(filename, "w");
    if (!data_output_file) {
        perror("Failed to create data output file");
        return;
    }
    
    // Write header
    fprintf(data_output_file, "point_index,timestamp_us,");
    
    // Plan positions header
    for (int i = 0; i < arm_dof_angle; i++) {
        fprintf(data_output_file, "plan_pos_%d,", i);
    }
    
    // Actual positions header
    for (int i = 0; i < arm_dof_angle; i++) {
        fprintf(data_output_file, "actual_pos_%d,", i);
    }
    
    // Current header
    for (int i = 0; i < arm_dof_angle; i++) {
        if (i < arm_dof_angle - 1) {
            fprintf(data_output_file, "current_%d,", i);
        } else {
            fprintf(data_output_file, "current_%d\n", i);
        }
    }
    
    is_recording = true;
    printf("Data recording started: %s\n", filename);
}

// Function to close data file
void stop_data_recording() {
    if (data_output_file) {
        fclose(data_output_file);
        data_output_file = NULL;
        is_recording = false;
        printf("Data recording stopped\n");
    }
}

// Function to save current data point with microsecond timestamp
void save_joint_data(int point_index, float* plan_positions, rm_realtime_arm_joint_state_t* data) {
    if (!is_recording || !data_output_file) {
        return;
    }
    
    // Get current timestamp in microseconds
    unsigned long long timestamp_us = 0;
    
    #ifdef _WIN32
    // Windows implementation for microsecond precision
    LARGE_INTEGER frequency, count;
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&count);
    // Convert to microseconds
    timestamp_us = (count.QuadPart * 1000000) / frequency.QuadPart;
    #else
    // Linux implementation with microsecond precision
    struct timeval tv;
    gettimeofday(&tv, NULL);
    timestamp_us = (unsigned long long)tv.tv_sec * 1000000 + tv.tv_usec;
    #endif
    
    // Write point index and timestamp (now in microseconds)
    fprintf(data_output_file, "%d,%llu,", point_index, timestamp_us);
    
    // Write planned positions
    for (int i = 0; i < arm_dof_angle; i++) {
        fprintf(data_output_file, "%.6f,", plan_positions[i]);
    }
    
    // Write actual positions
    for (int i = 0; i < arm_dof_angle; i++) {
        fprintf(data_output_file, "%.6f,", data->joint_status.joint_position[i]);
    }
    
    // Write current values
    for (int i = 0; i < arm_dof_angle; i++) {
        if (i < arm_dof_angle - 1) {
            fprintf(data_output_file, "%.6f,", data->joint_status.joint_current[i]);
        } else {
            fprintf(data_output_file, "%.6f\n", data->joint_status.joint_current[i]);
        }
    }
    
    // Flush to ensure data is written even if program crashes
    fflush(data_output_file);
}

// Define the function arm_state_return

void custom_api_log(const char* message, va_list args) {
    if (!message) {
        fprintf(stderr, "Error: message is a null pointer\n");
        return;
    }

    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), message, args);
    printf(" %s\n",  buffer);
}

void callback_rm_realtime_arm_joint_state(rm_realtime_arm_joint_state_t data) {
    printf("==================MYCALLBACK==================\n");
    printf("Current state: %.3f\n", data.joint_status.joint_current[0]);
    printf("Current angles: %.3f %.3f %.3f %.3f %.3f %.3f\n", data.joint_status.joint_position[0], data.joint_status.joint_position[1],
           data.joint_status.joint_position[2], data.joint_status.joint_position[3], data.joint_status.joint_position[4], data.joint_status.joint_position[5]);

    printf("Current state: %u\n", data.joint_status.joint_err_code[0]);
    // Handle received data
    printf("Error Code: %d\n", data.errCode);
    printf("Arm IP: %s\n", data.arm_ip);
    
    printf("Error: ");
    for (int i = 0; i < data.err.err_len; i++) {
        printf("%04x ", data.err.err[i]);
    }
    printf("\n");

    // Handle joint state
    printf("Joint Position:\n");
    for (int i = 0; i < 6; i++) {
        printf(" %.3f \n", data.joint_status.joint_position[i]);
    }

    // Handle waypoint information
    printf("Waypoint:\n");
    printf("  Euler: [%.3f, %.3f, %.3f]\n", data.waypoint.euler.rx, data.waypoint.euler.ry, data.waypoint.euler.rz);
    printf("  Position: [%.3f, %.3f, %.3f]\n", data.waypoint.position.x, data.waypoint.position.y, data.waypoint.position.z);
    printf("  Quat: [%.3f, %.3f, %.3f, %.3f]\n", data.waypoint.quaternion.w, data.waypoint.quaternion.x, data.waypoint.quaternion.y, data.waypoint.quaternion.z);
    
    // Save data to file if recording is active
    if (is_recording && current_point_index >= 0) {
        save_joint_data(current_point_index, current_plan_point, &data);
    }
}

void demo_movej_canfd(rm_robot_handle* handle) {
    printf("Trying to open file: %s\n", DATA_FILE_PATH);

    FILE* file = fopen(DATA_FILE_PATH, "r");
    if (!file) {
        perror("Failed to open file");
        return;
    }

    float points[MAX_POINTS][arm_dof_angle];
    int point_count = 0;
    while (fscanf(file, "%f,%f,%f,%f,%f,%f,%f",
                  &points[point_count][0], &points[point_count][1], &points[point_count][2],
                  &points[point_count][3], &points[point_count][4], &points[point_count][5],
                  &points[point_count][6]) == arm_dof_angle) {
        point_count++;
    }
    fclose(file);
    if (point_count == 0) {
        printf("No valid points data found in file\n");
        return;
    }
    rm_robot_info_t robot_info;
    int info_result = robotic_arm.rm_get_robot_info(handle, &robot_info);
    if (info_result != 0) {
        printf("Failed to get robot info\n");
        return;
    }

    int dof = robot_info.arm_dof;
    if (dof != 6 && dof != 7) {
        printf("Invalid degree of freedom, must be 6 or 7\n");
        return;
    }

    if (point_count == 0 || (point_count > 0 && dof != arm_dof_angle)) {
        printf("Invalid points data in file\n");
        return;
    }

    printf("Total points: %d\n", point_count);
    int movej_result = robotic_arm.rm_movej(handle, points[0], 20, 0, RM_TRAJECTORY_DISCONNECT_E, RM_MOVE_MULTI_BLOCK);
    if (movej_result != 0) {
        printf("movej failed with error code: %d\n", movej_result);
    }
    
    rm_movej_canfd_mode_t param = {0};
    param.follow = false;
    param.expand = 0;
    robotic_arm.rm_realtime_arm_state_call_back(callback_rm_realtime_arm_joint_state);
    
    // Start data recording (will only start if switch is enabled)
    start_data_recording();
    
    for (int i = 0; i < point_count; ++i) {
        printf("Moving to point %d\n", i);
        
        int result = robotic_arm.rm_movej_canfd(handle, points[i], true, 0, 0);
        if (result != 0) {
            printf("Error at point %d: %d\n", i, result);
        }

        // Update current plan point and index for data recording
        memcpy(current_plan_point, points[i], arm_dof_angle * sizeof(float));
        current_point_index = i;
        
        SLEEP_MS(2);
    }

    SLEEP_S(5);
    // Stop data recording
    stop_data_recording();
    
    printf("Pass-through completed\n");
    SLEEP_S(2);

    // float *home_position = (float *)malloc(dof * sizeof(float));

    // for (int i = 0; i < dof; ++i) {
    //     home_position[i] = 0.0f;
    // }
    // int movej_ret = robotic_arm.rm_movej(handle, home_position, 20, 0, RM_TRAJECTORY_DISCONNECT_E, RM_MOVE_MULTI_BLOCK);
    // printf("movej_cmd joint movement 1: %d\n", movej_ret);
    // SLEEP_S(2);
    
    // free(home_position);
}

int main(int argc, char *argv[]) {
    int result = -1;

    // Set data recording switch (1 = enabled, 0 = disabled)
    // Change this value to control data recording
    data_recording_enabled = 1;
    
    robotic_arm.rm_set_log_call_back(custom_api_log, 3);
    result = robotic_arm.rm_init(RM_TRIPLE_MODE_E);
    if (result != 0) {
        printf("Initialization failed with error code: %d.\n", result);
        return -1;
    }

    char *api_version = robotic_arm.rm_api_version();
    printf("API Version: %s.\n", api_version);

    const char *robot_ip_address = "192.168.1.18";
    int robot_port = 8080;
    rm_robot_handle *robot_handle = robotic_arm.rm_create_robot_arm(robot_ip_address, robot_port);
    if (robot_handle == NULL) {
        printf("Failed to create robot handle.\n");
        return -1;
    } else {
        printf("Robot handle created successfully: %d\n", robot_handle->id);
    }

    rm_realtime_push_config_t config = {1, true, 8089, 0, "192.168.1.88"};
    result = robotic_arm.rm_set_realtime_push(robot_handle, config);
    if (result != 0) {
        printf("Failed to set realtime push configuration, error code: %d\n", result);
    } else {
        printf("Successfully set realtime push configuration.\n");
    }

    demo_movej_canfd(robot_handle);
    // Disconnect the robot arm
    result = robotic_arm.rm_delete_robot_arm(robot_handle);
    if(result != 0)
    {
        return -1;
    }

    return 0;
}
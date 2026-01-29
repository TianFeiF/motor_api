/*
 * test_interactive.c
 * 
 * Interactive control test for Slaves 0:0 ~ 0:6
 * - Slave 0: INEXBOT IO (Toggle with q/e)
 * - Slave 1-3: HCFA Servos
 * - Slave 4-6: Hans Robot Servos
 * 
 * Controls:
 * - 'q': IO Off (Axis 0)
 * - 'e': IO On (Axis 0)
 * - '1'-'9': Select Servo Axis
 * - Left/Right Arrow: Jog Selected Axis
 * - ESC: Exit
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <time.h>
#include <string.h>
#include <pthread.h>

// We include internal header to inspect axis types dynamically
#include "motor_api_internal.h"

// Helper for non-blocking keyboard input
static int kbhit(void) {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}

static int getch(void) {
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

// Global state
static volatile int g_selected_servo_idx = 0; // Index into servo_axis_map
static int g_servo_axis_map[32]; // Maps logical index 0..N to actual axis_idx
static int g_servo_count = 0;
static struct motor_api_handle *g_handle = NULL;
static int g_io_axis_idx = -1;

void *control_loop(void *arg) {
    while (1) {
        motor_api_run_once(g_handle);
        usleep(3500); // 4ms cycle approx
    }
    return NULL;
}

int main(int argc, char **argv) {
    // 1. Find ENI
    char eni_path[512];
    if (motor_api_find_latest_eni_xml("doc", eni_path, sizeof(eni_path)) != MA_OK) {
        // Fallback to absolute path if running from build
        if (motor_api_find_latest_eni_xml("../doc", eni_path, sizeof(eni_path)) != MA_OK) {
             fprintf(stderr, "Failed to find ENI file in doc/\n");
             return 1;
        }
    }
    printf("Using ENI: %s\n", eni_path);

    // 2. Create Motor API
    uint16_t slave_count = 0;
    if (motor_api_create(eni_path, 4000, &slave_count, &g_handle) != MA_OK) {
        fprintf(stderr, "Failed to create motor api handle\n");
        return 1;
    }
    
    // 3. Scan Axes
    // Cast to internal handle to inspect
    motor_api_handle_t *h = (motor_api_handle_t *)g_handle;
    
    printf("Scanning Axes...\n");
    for (int i = 0; i < h->axis_count; ++i) {
        ma_axis_data_t *ax = &h->axes[i];
        if (ax->type == MA_AXIS_TYPE_IO) {
            // Assume the first IO axis is the one we want to control (Slave 0)
            if (g_io_axis_idx == -1) g_io_axis_idx = i;
            printf("  [Axis %d] IO (Slave %d)\n", i, ax->slave_idx);
        } else if (ax->type == MA_AXIS_TYPE_CIA402) {
            if (g_servo_count < 32) {
                g_servo_axis_map[g_servo_count++] = i;
                printf("  [Axis %d] Servo %d (Slave %d)\n", i, g_servo_count, ax->slave_idx);
            }
        }
    }

    if (g_io_axis_idx == -1) {
        printf("Warning: No IO axis found.\n");
    }

    // 4. Start Control Thread
    pthread_t tid;
    pthread_create(&tid, NULL, control_loop, NULL);

    printf("\n=== Controls ===\n");
    printf(" [q] IO Low (Axis %d)\n", g_io_axis_idx);
    printf(" [e] IO High (Axis %d)\n", g_io_axis_idx);
    printf(" [1-%d] Select Servo Axis\n", g_servo_count);
    printf(" [<-] Jog Reverse\n");
    printf(" [->] Jog Forward\n");
    printf(" [Space] Stop Jog\n");
    printf(" [ESC] Quit\n");

    int cmd_step = 10000; // pulses per cycle

    while (1) {
        if (kbhit()) {
            int ch = getch();
            if (ch == 27) { // ESC
                // Check for Arrow Keys (ESC [ C / ESC [ D)
                int next1 = kbhit() ? getch() : 0;
                if (next1 == 91) { // '['
                    int next2 = getch();
                    int axis = g_servo_axis_map[g_selected_servo_idx];
                    if (next2 == 68) { // Left Arrow
                        printf("\rAxis %d: Jog Reverse   ", axis);
                        motor_api_set_axis_command(g_handle, axis, true, -1, cmd_step);
                    } else if (next2 == 67) { // Right Arrow
                        printf("\rAxis %d: Jog Forward   ", axis);
                        motor_api_set_axis_command(g_handle, axis, true, 1, cmd_step);
                    }
                } else {
                    break; // Real ESC
                }
            } else if (ch == 'q') {
                if (g_io_axis_idx >= 0) {
                    printf("\rIO Axis %d: LOW        ", g_io_axis_idx);
                    motor_api_set_io_output(g_handle, g_io_axis_idx, 0x00000000);
                }
            } else if (ch == 'e') {
                if (g_io_axis_idx >= 0) {
                    printf("\rIO Axis %d: HIGH       ", g_io_axis_idx);
                    motor_api_set_io_output(g_handle, g_io_axis_idx, 0xFFFFFFFF);
                }
            } else if (ch >= '1' && ch <= '9') {
                int idx = ch - '1';
                if (idx < g_servo_count) {
                    g_selected_servo_idx = idx;
                    printf("\rSelected Servo: %d (Axis %d) ", idx+1, g_servo_axis_map[idx]);
                }
            } else if (ch == ' ') {
                int axis = g_servo_axis_map[g_selected_servo_idx];
                printf("\rAxis %d: Stop          ", axis);
                motor_api_set_axis_command(g_handle, axis, false, 0, 0);
            }
            fflush(stdout);
        }
        usleep(10000);
    }

    motor_api_destroy(g_handle);
    return 0;
}

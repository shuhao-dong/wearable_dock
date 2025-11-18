/*
 * wearable_dock.c: exFAT logs extractor + IMU to JSON to MQTT
 *
 * Compile:
 *   cc -Wall -O2 wearable_dock.c -ludev -lmosquitto -o wearable_dock_run
 */

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <libudev.h>
#include <mosquitto.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>
#include <dirent.h>
#include <limits.h>

#ifndef PATH_MAX
#define PATH_MAX 4096
#endif

/* USB ID of your wearable MSC device */
#define WEARABLE_VENDOR_HEX "0001"
#define WEARABLE_PRODUCT_HEX "0001"

/* Where we mount the wearable’s exFAT volume (no GUI / spaces) */
#define MOUNT_POINT "/mnt/wearable"

/* Where to store offloaded sessions */
#define SESSIONS_BASE "/home/torus-4/wearable_dock/extracted"
#define ARCHIVE_BASE SESSIONS_BASE "/archive"

/* Subdirectory created by firmware on the card */
#define LOGS_SUBDIR "logs"

/* MQTT configuration */
#define MQTT_HOST "192.168.88.251"
#define MQTT_PORT 1883
#define MQTT_TOPIC "BORUS/extf"

/* Binary record from firmware:
 *   uint32_t timestamp_ms;
 *   uint32_t pressure_pa;
 *   int16_t  imu[6];
 */
#define RECORD_SIZE (4 + 4 + 6 * 2)
#define IMU_SCALE 100.0f

static volatile sig_atomic_t quit_flag = 0;

/* ======================== SIGNAL HANDLER ========================= */

static void handle_sigint(int sig)
{
    (void)sig;
    quit_flag = 1;
}

/* ========================= SMALL HELPERS ========================= */

static int join_path(const char *a, const char *b, char *out, size_t out_sz)
{
    int n = snprintf(out, out_sz, "%s/%s", a, b);
    if (n < 0 || (size_t)n >= out_sz)
    {
        errno = ENAMETOOLONG;
        return -1;
    }
    return 0;
}

static int ensure_dir(const char *path)
{
    if (mkdir(path, 0755) == -1)
    {
        if (errno == EEXIST)
        {
            return 0;
        }
        perror(path);
        return -1;
    }
    return 0;
}

static int run_child(char *const argv[])
{
    pid_t pid = fork();
    if (pid < 0)
    {
        perror("fork");
        return -1;
    }
    if (pid == 0)
    {
        execvp(argv[0], argv);
        _exit(127);
    }
    int st;
    if (waitpid(pid, &st, 0) < 0)
    {
        perror("waitpid");
        return -1;
    }
    if (WIFEXITED(st))
    {
        return WEXITSTATUS(st);
    }
    return -1;
}

static int make_session_dir(char *session_dir, size_t sz)
{
    time_t now = time(NULL);
    struct tm tm;
    localtime_r(&now, &tm);

    char stamp[32];
    if (!strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tm))
    {
        fprintf(stderr, "strftime failed\n");
        return -1;
    }

    if (ensure_dir(SESSIONS_BASE) != 0)
    {
        return -1;
    }

    int n = snprintf(session_dir, sz, "%s/%s", SESSIONS_BASE, stamp);
    if (n < 0 || (size_t)n >= sz)
    {
        errno = ENAMETOOLONG;
        return -1;
    }

    if (ensure_dir(session_dir) != 0)
    {
        return -1;
    }
    return 0;
}

/* ====================== MOUNT / UNMOUNT HELPERS ================== */

static void ensure_unmounted(const char *mp)
{
    char *av[] = {"umount", (char *)mp, NULL};
    (void)run_child(av); /* ignore errors */
}

static int mount_exfat(const char *disk_devnode, char *out_dev, size_t out_sz)
{
    struct stat st;
    char dev_to_mount[PATH_MAX];

    /* Start with the base node, e.g. "/dev/sda" */
    if (snprintf(dev_to_mount, sizeof(dev_to_mount), "%s", disk_devnode) >= (int)sizeof(dev_to_mount))
    {
        fprintf(stderr, "disk_devnode too long: %s\n", disk_devnode);
        return -1;
    }

    /* Try partition "1": "/dev/sda1" (safe append) */
    size_t len = strlen(dev_to_mount);
    if (len + 1 < sizeof(dev_to_mount))
    { /* +1 for '1', +1 for '\0' */
        dev_to_mount[len] = '1';
        dev_to_mount[len + 1] = '\0';
    }
    else
    {
        /* Name is already too long to add '1' safely, fall back to whole disk */
        fprintf(stderr, "Cannot append '1' to %s safely, using whole disk\n", dev_to_mount);
    }

    /* If "/dev/sda1" (or whatever) doesn’t exist, fall back to the base disk node */
    if (stat(dev_to_mount, &st) != 0)
    {
        /* fallback: just use disk_devnode as-is */
        if (snprintf(dev_to_mount, sizeof(dev_to_mount), "%s", disk_devnode) >= (int)sizeof(dev_to_mount))
        {
            fprintf(stderr, "disk_devnode too long (fallback): %s\n", disk_devnode);
            return -1;
        }
    }

    if (ensure_dir(MOUNT_POINT) != 0)
    {
        return -1;
    }

    ensure_unmounted(MOUNT_POINT);

    char *av[] = {"mount", "-t", "exfat", dev_to_mount, MOUNT_POINT, NULL};
    int rc = run_child(av);
    if (rc != 0)
    {
        fprintf(stderr, "mount exfat %s -> %s failed (rc=%d)\n",
                dev_to_mount, MOUNT_POINT, rc);
        return -1;
    }

    if (out_dev && out_sz > 0)
    {
        strncpy(out_dev, dev_to_mount, out_sz);
        out_dev[out_sz - 1] = '\0';
    }

    return 0;
}

/* ===================== COPY + DELETE LOG FILES =================== */

static int copy_file(const char *src, const char *dst)
{
    FILE *in = fopen(src, "rb");
    if (!in)
    {
        fprintf(stderr, "Failed to open %s for read: %s\n", src, strerror(errno));
        return -1;
    }

    FILE *out = fopen(dst, "wb");
    if (!out)
    {
        fprintf(stderr, "Failed to open %s for write: %s\n", dst, strerror(errno));
        fclose(in);
        return -1;
    }

    uint8_t buf[4096];
    size_t n;
    int rc = 0;

    while ((n = fread(buf, 1, sizeof(buf), in)) > 0)
    {
        if (fwrite(buf, 1, n, out) != n)
        {
            fprintf(stderr, "Write error to %s: %s\n", dst, strerror(errno));
            rc = -1;
            break;
        }
    }

    if (ferror(in))
    {
        fprintf(stderr, "Read error from %s\n", src);
        rc = -1;
    }

    fclose(in);
    if (fclose(out) != 0)
    {
        fprintf(stderr, "Close error on %s: %s\n", dst, strerror(errno));
        rc = -1;
    }

    return rc;
}

/* Copy all *.BIN / *.bin from src_logs into dest_logs and delete them on card */
static int copy_and_delete_logs(const char *src_logs, const char *dest_logs)
{
    if (ensure_dir(dest_logs) != 0)
    {
        return -1;
    }

    DIR *dir = opendir(src_logs);
    if (!dir)
    {
        fprintf(stderr, "Cannot open logs directory %s: %s\n",
                src_logs, strerror(errno));
        return -1;
    }

    struct dirent *de;
    int copied = 0;

    while ((de = readdir(dir)) != NULL)
    {
        if (de->d_name[0] == '.')
        {
            continue;
        }

        const char *dot = strrchr(de->d_name, '.');
        if (!dot || (strcmp(dot, ".BIN") != 0 && strcmp(dot, ".bin") != 0))
        {
            continue; /* ignore non-BIN files */
        }

        char src_path[PATH_MAX];
        char dst_path[PATH_MAX];

        if (join_path(src_logs, de->d_name, src_path, sizeof(src_path)) != 0 ||
            join_path(dest_logs, de->d_name, dst_path, sizeof(dst_path)) != 0)
        {
            fprintf(stderr, "Path too long for %s\n", de->d_name);
            continue;
        }

        printf("  Copying %s -> %s\n", src_path, dst_path);
        if (copy_file(src_path, dst_path) == 0)
        {
            ++copied;
            if (unlink(src_path) != 0)
            {
                fprintf(stderr, "  Warning: failed to delete %s: %s\n",
                        src_path, strerror(errno));
            }
            else
            {
                printf("  Deleted %s from wearable\n", src_path);
            }
        }
    }

    closedir(dir);

    if (copied == 0)
    {
        printf("No .BIN files found in %s\n", src_logs);
    }
    else
    {
        printf("Copied %d log file(s) from wearable.\n", copied);
    }

    return 0;
}

/* ======================== RECORD DECODE + MQTT =================== */

static void decode_record(const uint8_t buf[RECORD_SIZE],
                          uint32_t *timestamp_ms,
                          float *pressure_pa,
                          float acc[3],
                          float gyr[3])
{
    uint32_t ts;
    uint32_t p;
    int16_t raw[6];

    memcpy(&ts, buf, 4);
    memcpy(&p, buf + 4, 4);

    for (int i = 0; i < 6; i++)
    {
        memcpy(&raw[i], buf + 8 + 2 * i, 2);
    }

    *timestamp_ms = ts;
    *pressure_pa = p / 100.0f;

    acc[0] = raw[0] / IMU_SCALE;
    acc[1] = raw[1] / IMU_SCALE;
    acc[2] = raw[2] / IMU_SCALE;

    gyr[0] = raw[3] / IMU_SCALE;
    gyr[1] = raw[4] / IMU_SCALE;
    gyr[2] = raw[5] / IMU_SCALE;
}

/* session_root is e.g. /home/.../extracted/20251118_102030 */
static int convert_and_publish(const char *session_root)
{
    char logs_dir[PATH_MAX];

    if (join_path(session_root, LOGS_SUBDIR, logs_dir, sizeof(logs_dir)) != 0)
    {
        fprintf(stderr, "Path too long for logs_dir\n");
        return -1;
    }

    DIR *dir = opendir(logs_dir);
    if (!dir)
    {
        fprintf(stderr, "convert_and_publish: cannot open %s: %s\n",
                logs_dir, strerror(errno));
        return -1;
    }

    /* Setup MQTT */
    mosquitto_lib_init();
    struct mosquitto *m = mosquitto_new(NULL, true, NULL);
    if (!m)
    {
        fprintf(stderr, "mosquitto_new failed\n");
        closedir(dir);
        mosquitto_lib_cleanup();
        return -1;
    }

    int rc = mosquitto_connect_async(m, MQTT_HOST, MQTT_PORT, 60);
    if (rc != MOSQ_ERR_SUCCESS)
    {
        fprintf(stderr, "mosquitto_connect failed: %s\n",
                mosquitto_strerror(rc));
        mosquitto_destroy(m);
        closedir(dir);
        mosquitto_lib_cleanup();
        return -1;
    }

    mosquitto_loop_start(m);

    struct dirent *de;
    int total_files = 0;
    int total_records = 0;

    while ((de = readdir(dir)) != NULL)
    {
        if (de->d_name[0] == '.')
        {
            continue;
        }

        const char *dot = strrchr(de->d_name, '.');
        if (!dot || (strcmp(dot, ".BIN") != 0 && strcmp(dot, ".bin") != 0))
        {
            continue;
        }

        char file_path[PATH_MAX];
        if (join_path(logs_dir, de->d_name, file_path, sizeof(file_path)) != 0)
        {
            fprintf(stderr, "Path too long for %s\n", de->d_name);
            continue;
        }

        FILE *fp = fopen(file_path, "rb");
        if (!fp)
        {
            fprintf(stderr, "Failed to open %s: %s\n", file_path, strerror(errno));
            continue;
        }

        printf("Decoding %s ...\n", file_path);
        ++total_files;

        uint8_t buf[RECORD_SIZE];
        size_t n;

        while ((n = fread(buf, 1, RECORD_SIZE, fp)) == RECORD_SIZE)
        {
            uint32_t ts_ms;
            float p_pa, acc[3], gyr[3];

            decode_record(buf, &ts_ms, &p_pa, acc, gyr);

            char payload[256];
            int len = snprintf(payload, sizeof(payload),
                               "{\"timestamp_ms\":%u,"
                               "\"pressure_pa\":%.2f,"
                               "\"acceleration\":[%.2f,%.2f,%.2f],"
                               "\"gyroscope\":[%.2f,%.2f,%.2f]}",
                               ts_ms, p_pa,
                               acc[0], acc[1], acc[2],
                               gyr[0], gyr[1], gyr[2]);

            if (len < 0 || (size_t)len >= sizeof(payload))
            {
                fprintf(stderr, "Payload truncated for timestamp %u\n", ts_ms);
                continue;
            }

            if (len > 0 && (size_t)len < sizeof(payload))
            {
                /* Print the JSON we are about to publish */
                printf("MQTT JSON -> %s\n", payload);
                fflush(stdout); /* helpful if running under systemd */
            }

            rc = mosquitto_publish(m, NULL, MQTT_TOPIC,
                                   (int)len, payload, 0, false);
            if (rc != MOSQ_ERR_SUCCESS)
            {
                fprintf(stderr, "mosquitto_publish failed: %s\n",
                        mosquitto_strerror(rc));
            }
            else
            {
                ++total_records;
            }
        }

        if (ferror(fp))
        {
            fprintf(stderr, "Read error in %s\n", file_path);
        }

        fclose(fp);
    }

    closedir(dir);

    mosquitto_loop_stop(m, true);
    mosquitto_disconnect(m);
    mosquitto_destroy(m);
    mosquitto_lib_cleanup();

    printf("Published %d records from %d file(s) for session %s\n",
           total_records, total_files, session_root);

    return 0;
}

/* ============================= ARCHIVE =========================== */

static int archive_session(const char *session_root)
{
    if (ensure_dir(ARCHIVE_BASE) != 0)
    {
        return -1;
    }

    const char *name = strrchr(session_root, '/');
    name = name ? name + 1 : session_root;

    char dst[PATH_MAX];
    if (join_path(ARCHIVE_BASE, name, dst, sizeof(dst)) != 0)
    {
        fprintf(stderr, "archive path too long\n");
        return -1;
    }

    if (rename(session_root, dst) != 0)
    {
        fprintf(stderr, "Failed to move %s -> %s: %s\n",
                session_root, dst, strerror(errno));
        return -1;
    }

    printf("Archived session to %s\n", dst);
    return 0;
}

/* ============================= UDEV WAIT ========================= */

static int wait_for_device(struct udev_monitor *mon,
                           const char *target_action,
                           char *out_devnode,
                           size_t out_sz)
{
    int fd = udev_monitor_get_fd(mon);
    struct pollfd fds[1] = {
        {.fd = fd, .events = POLLIN}};

    for (;;)
    {
        if (quit_flag)
        {
            return -1;
        }

        int ret = poll(fds, 1, -1);
        if (ret < 0)
        {
            if (errno == EINTR && quit_flag)
            {
                return -1;
            }
            if (errno == EINTR)
            {
                continue;
            }
            perror("poll");
            return -1;
        }

        if (!(fds[0].revents & POLLIN))
        {
            continue;
        }

        struct udev_device *dev = udev_monitor_receive_device(mon);
        if (!dev)
        {
            continue;
        }

        const char *action = udev_device_get_action(dev);
        const char *subsys = udev_device_get_subsystem(dev);
        const char *devtype = udev_device_get_devtype(dev);

        const char *vid = udev_device_get_property_value(dev, "ID_VENDOR_ID");
        const char *pid = udev_device_get_property_value(dev, "ID_MODEL_ID");
        const char *node = udev_device_get_devnode(dev);

        if (action && !strcmp(action, target_action) &&
            subsys && !strcmp(subsys, "block") &&
            devtype && !strcmp(devtype, "disk") &&
            vid && pid &&
            !strcasecmp(vid, WEARABLE_VENDOR_HEX) &&
            !strcasecmp(pid, WEARABLE_PRODUCT_HEX))
        {

            printf("  udev: %s event for %s (VID=%s PID=%s)\n",
                   target_action,
                   node ? node : "(unknown)",
                   vid, pid);

            if (out_devnode && out_sz > 0 && node)
            {
                strncpy(out_devnode, node, out_sz);
                out_devnode[out_sz - 1] = '\0';
            }

            udev_device_unref(dev);
            return 0;
        }

        udev_device_unref(dev);
    }
}

/* ============================= HANDLER =========================== */

static void handle_device(const char *disk_devnode)
{
    /* 1) Mount exFAT from this disk */
    char mounted_dev[PATH_MAX];
    if (mount_exfat(disk_devnode, mounted_dev, sizeof(mounted_dev)) != 0)
    {
        fprintf(stderr, "Failed to mount %s as exFAT\n", disk_devnode);
        return;
    }

    /* 2) Wait for /mnt/wearable/logs to exist */
    char src_logs[PATH_MAX];
    if (join_path(MOUNT_POINT, LOGS_SUBDIR, src_logs, sizeof(src_logs)) != 0)
    {
        fprintf(stderr, "src_logs path too long\n");
        ensure_unmounted(MOUNT_POINT);
        return;
    }

    struct stat st;
    int tries = 50; /* ~5s */
    while (tries-- > 0)
    {
        if (stat(src_logs, &st) == 0 && S_ISDIR(st.st_mode))
        {
            break;
        }
        usleep(100 * 1000);
    }
    if (tries <= 0)
    {
        fprintf(stderr, "Timed out waiting for %s\n", src_logs);
        ensure_unmounted(MOUNT_POINT);
        return;
    }

    /* 3) Prepare destination session directory */
    char session_dir[PATH_MAX];
    if (make_session_dir(session_dir, sizeof(session_dir)) != 0)
    {
        fprintf(stderr, "Failed to create session directory\n");
        ensure_unmounted(MOUNT_POINT);
        return;
    }

    char dest_logs[PATH_MAX];
    if (join_path(session_dir, LOGS_SUBDIR, dest_logs, sizeof(dest_logs)) != 0)
    {
        fprintf(stderr, "dest_logs path too long\n");
        ensure_unmounted(MOUNT_POINT);
        return;
    }

    printf("Session dir: %s\n", session_dir);

    /* 4) Copy + delete log files from wearable */
    if (copy_and_delete_logs(src_logs, dest_logs) != 0)
    {
        fprintf(stderr, "Error copying log files\n");
    }

    /* 5) Unmount as early as possible */
    ensure_unmounted(MOUNT_POINT);

    /* 6) Decode + publish over MQTT */
    convert_and_publish(session_dir);

    /* 7) Archive session folder */
    archive_session(session_dir);
}

/* =============================== MAIN ============================ */

int main(void)
{
    signal(SIGINT, handle_sigint);
    signal(SIGTERM, handle_sigint);

    struct udev *udev = udev_new();
    if (!udev)
    {
        fprintf(stderr, "udev_new() failed\n");
        return 1;
    }

    struct udev_monitor *mon =
        udev_monitor_new_from_netlink(udev, "udev");
    if (!mon)
    {
        fprintf(stderr, "udev_monitor_new_from_netlink() failed\n");
        udev_unref(udev);
        return 1;
    }

    udev_monitor_filter_add_match_subsystem_devtype(mon, "block", NULL);
    udev_monitor_enable_receiving(mon);

    char disk_devnode[PATH_MAX];

    while (!quit_flag)
    {
        printf("Waiting for USB %s:%s ...\n",
               WEARABLE_VENDOR_HEX, WEARABLE_PRODUCT_HEX);

        if (wait_for_device(mon, "add",
                            disk_devnode, sizeof(disk_devnode)) != 0)
        {
            if (quit_flag)
                break;
            fprintf(stderr, "wait_for_device(add) failed\n");
            break;
        }
        if (quit_flag)
            break;

        printf("Wearable detected - processing\n");
        handle_device(disk_devnode);

        printf("Waiting for removal ...\n");
        if (wait_for_device(mon, "remove", NULL, 0) != 0)
        {
            if (quit_flag)
                break;
            fprintf(stderr, "wait_for_device(remove) failed\n");
            break;
        }
        printf("Device removed, ready for next.\n");
    }

    udev_monitor_unref(mon);
    udev_unref(udev);
    return 0;
}

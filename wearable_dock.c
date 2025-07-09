/*
 * wearable_dock.c: DFU + LittleFS extractor + IMU to JSON to MQTT
 *
 * Compile:
 *   cc -Wall -O2 wearable_dock.c -ludev -lmosquitto -o ~/wearable_dock_run
 *
 * Needs: libudev-dev, FUSE, littlefs-fuse (lfs), dfu-util, libmosquitto-dev
 */

#define _GNU_SOURCE
#include <fcntl.h>
#include <ftw.h>
#include <libudev.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/statfs.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>
#include <dirent.h>
#include <mosquitto.h>
#include <errno.h>

#ifndef PATH_MAX
#define PATH_MAX 4096
#endif
#ifndef FUSE_SUPER_MAGIC
#define FUSE_SUPER_MAGIC 0x65735546
#endif

/* Runtime configuration */
#define VENDOR_ID "0001"
#define PRODUCT_ID "0001"

#define LFS_BIN "/home/torus-pi5/littlefs-fuse/lfs"
#define LFS_ARGS "--block_count=1760 --block_size=4096 --read_size=16 " \
                 "--prog_size=16 --cache_size=64 --lookahead_size=32"

#define MOUNT_POINT "/mnt/wearable"
#define DEST_BASE "/home/torus-pi5/wearable_dock/extracted"
#define ARCHIVE_DIR DEST_BASE "/archive"

#define DFU_UTIL "/usr/bin/dfu-util"
#define FW_DIR "/home/torus-pi5/wearable_dock/new_firmware"
#define FW_ARCHIVE "/home/torus-pi5/wearable_dock/new_firmware/archive"

#define BIN_NAME "imu_log.bin"
#define BROKER_ADDR "192.168.88.251"
#define BROKER_PORT 1883
#define MQTT_TOPIC "BORUS/extf"

#define RECORD_SIZE (4 + 6 * 2)
#define SCALE 100.0f

static int debouncing = 0;
static long remove_seen_at_ms = 0;
static char current_usb_path[PATH_MAX] = "";
static volatile sig_atomic_t lfs_pid = -1;
static volatile sig_atomic_t quit_flag = 0;

/**
 * @brief Construct the full path to the destination
 *
 * Combines two paths to a single destination path
 *
 * @param a Pointer to path a
 * @param b Pointer to path b
 * @param out Pointer to the destination path
 *
 * @return 0 on success -1 on failure
 */
static int path_join(const char *a, const char *b, char *out)
{
    size_t la = strlen(a), lb = strlen(b);
    if (la + 1 + lb >= PATH_MAX)
    {
        return -1;
    }
    memcpy(out, a, la);
    out[la] = '/';
    memcpy(out + la + 1, b, lb);
    out[la + 1 + lb] = '\0';
    return 0;
}

/**
 * @brief Get the current time in ms
 */
static long now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

/**
 * @brief Wait for file size to remain unchanged
 *
 * This function will check if the file size provided in the directory remain unchanged before
 * proceeding to the next operation. This prevent copying empty file for further processing.
 *
 * @param dir Directory to the file
 * @param fname File name of the file to check
 * @param timeout_ms Timeout in ms before
 */
static int wait_for_file(const char *dir, const char *fname, int timeout_ms)
{
    char p[PATH_MAX];
    if (path_join(dir, fname, p) < 0)
    {
        return -1;
    }

    long end = now_ms() + timeout_ms;
    struct stat st_prev = {0}, st_now;

    while (now_ms() < end)
    {
        if (stat(p, &st_now) == 0 && st_now.st_size > 0)
        {
            if (st_now.st_size == st_prev.st_size)
            {
                return 0;
            }
            st_prev = st_now;
        }
        usleep(200 * 1000); // Sleep for 200ms in between
    }
    return -1;
}

/**
 * @brief Run a subprocess using process spawning API
 *
 * This function will run a subprocess using fork/execvp family
 *
 * @param argv Process specific command
 *
 * @return 0 on success -1 on failure
 */
static int run_child(char *const argv[])
{
    /* Copy a child process */
    pid_t pid = fork();
    if (pid < 0)
    {
        return perror("fork"), -1;
    }

    if (pid == 0)
    {
        execvp(argv[0], argv);
        _exit(127);
    }

    int st;
    waitpid(pid, &st, 0);
    return WIFEXITED(st) ? WEXITSTATUS(st) : -1;
}

/**
 * @brief Collect child process that has finished
 *
 * This function prevents the creation of zombine processes by collecting every child
 * that has finished
 *
 */
static void reap(int sig)
{
    (void)sig;
    /* Check every child, non-blocking */
    while (waitpid(-1, NULL, WNOHANG) > 0)
    {
    }
}

/**
 * @brief Stop littlefs-fuse child process when required
 */
static void forward_quit(int sig)
{
    if (lfs_pid > 0)
    {
        kill(lfs_pid, sig);
    }

    quit_flag = sig;
}

static char src_root[PATH_MAX], dst_root[PATH_MAX];

/**
 * @brief Copy files
 */
static int copy_file(int in, int out)
{
    const size_t B = 256 * 1024;
    char *buf = malloc(B);
    if (!buf)
        return -1;
    ssize_t r;
    while ((r = read(in, buf, B)) > 0)
        if (write(out, buf, r) != r)
        {
            r = -1;
            break;
        }
    free(buf);
    return r < 0 ? -1 : 0;
}

/**
 * @brief Callback function when we decend the directory specified in @ref copy_tree
 */
static int cp_cb(const char *f, const struct stat *sb, int flag, struct FTW *p)
{
    (void)sb;
    (void)p;
    char dst[PATH_MAX];
    snprintf(dst, sizeof dst, "%s%s", dst_root, f + strlen(src_root));
    if (flag == FTW_D)
    {
        mkdir(dst, 0755);
        return 0;
    }
    else if (flag == FTW_F)
    {
        int in = open(f, O_RDONLY),
            out = open(dst, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (in < 0 || out < 0)
            return perror("open"), -1;
        int rc = copy_file(in, out);
        close(in);
        close(out);
        return rc;
    }
    return 0;
}

/**
 * @brief Decend along the path and copy everything
 *
 * @param src Path to the source directory
 * @param dst Path to the destination directory
 *
 * @return 0 on success -1 on failure
 */
static int copy_tree(const char *src, const char *dst)
{
    snprintf(src_root, sizeof src_root, "%s", src);
    snprintf(dst_root, sizeof dst_root, "%s", dst);
    return nftw(src, cp_cb, 16, FTW_PHYS);
}

/**
 * @brief Enumerate a block device after USB PID VID match
 *
 * This function will find a block device providing a match between
 * target PID VID and detected PID VID within specified time.
 *
 * @param u USB device
 * @param timeout_ms Timeout in ms
 *
 * @return the block device of match or NULL
 */
static char *find_block_dev(struct udev *u, int timeout_ms)
{
    long end = now_ms() + timeout_ms;
    while (now_ms() < end)
    {
        struct udev_enumerate *en = udev_enumerate_new(u);
        udev_enumerate_add_match_subsystem(en, "block");
        udev_enumerate_add_match_property(en, "DEVTYPE", "disk");
        udev_enumerate_scan_devices(en);
        struct udev_list_entry *list = udev_enumerate_get_list_entry(en), *e;
        udev_list_entry_foreach(e, list)
        {
            const char *path = udev_list_entry_get_name(e);
            struct udev_device *blk = udev_device_new_from_syspath(u, path);
            struct udev_device *usb = udev_device_get_parent_with_subsystem_devtype(
                blk, "usb", "usb_device");
            const char *vid = usb ? udev_device_get_sysattr_value(usb, "idVendor") : NULL;
            const char *pid = usb ? udev_device_get_sysattr_value(usb, "idProduct") : NULL;
            if (vid && pid && !strcmp(vid, VENDOR_ID) && !strcmp(pid, PRODUCT_ID))
            {
                const char *node = udev_device_get_devnode(blk);
                if (node)
                {
                    char *ret = strdup(node);
                    udev_enumerate_unref(en);
                    udev_device_unref(blk);
                    return ret;
                }
            }
            udev_device_unref(blk);
        }
        udev_enumerate_unref(en);
        usleep(250000);
    }
    return NULL;
}

/**
 * @brief Locate the new firmware for DFU
 *
 * This function will locate the new firmware under specified directory if there is any
 *
 * @return Path to the new firmware
 */
static char *next_firmware(void)
{
    DIR *d = opendir(FW_DIR);
    if (!d)
        return NULL;
    struct dirent *e;
    char path[PATH_MAX];
    while ((e = readdir(d)))
    {
        if (strstr(e->d_name, ".bin") && !strstr(e->d_name, ".bin.done"))
        {
            snprintf(path, sizeof path, "%s/%s", FW_DIR, e->d_name);
            closedir(d);
            return strdup(path);
        }
    }
    closedir(d);
    return NULL;
}

/**
 * @brief Perform the DFU proess for the wearable
 *
 * This function will reset device into DFU mode and download the new firmware to
 * the device if there is any
 *
 * @param serial Serial number of the device
 * @param bin Path to the new firmware.bin
 *
 * @return 0 on success -1 on failure
 */
static int perform_dfu(const char *bin)
{
    /* Download the new firmware to the device */
    char *av2[] = {(char *)DFU_UTIL, "-a", "1", "-t", "1024", "-D", (char *)bin, NULL};
    if (run_child(av2))
    {
        return fprintf(stderr, "DFU download failed\n"), -1;
    }

    /* Archive the downloaded firmware with download time */
    mkdir(FW_ARCHIVE, 0755);
    char arch[PATH_MAX];
    time_t t = time(NULL);
    struct tm tm;
    localtime_r(&t, &tm);
    strftime(arch, sizeof arch, FW_ARCHIVE "/%Y%m%d_%H%M%S.bin", &tm);
    if (rename(bin, arch))
    {
        perror("rename firmware");
        unlink(bin);
    }
    return 0;
}

/**
 * @brief Start a child process to mount the littlefs
 *
 * This is equivelant to ./lfs -f <block size etc.> /mnt/wearable. The -f option allows dock programme
 * to own the helper's PID to prevent zombine process.
 *
 * @param dev Device
 * @return PID of the child process or -1 if don't exist
 */
static pid_t start_lfs(const char *dev)
{
    pid_t pid = fork();
    if (pid < 0)
    {
        return -1;
    }

    if (!pid)
    {
        char *dup = strdup(LFS_ARGS);
        char *argv[32];
        int i = 0;
        argv[i++] = (char *)LFS_BIN;
        argv[i++] = "-f";
        argv[i++] = "-o"; 
        argv[i++] = "ro"; 
        for (char *t = strtok(dup, " "); t; t = strtok(NULL, " "))
        {
            argv[i++] = t;
        }
        argv[i++] = (char *)dev;
        argv[i++] = MOUNT_POINT;
        argv[i] = NULL;

        execv(argv[0], argv);
        _exit(127);
    }
    return pid;
}

/* Check if littls-fuse is mounted */
static int is_fuse_mounted(const char *mp)
{
    struct statfs s;
    return statfs(mp, &s) == 0 && s.f_type == FUSE_SUPER_MAGIC;
}

/* Unmount the mounted device */
static int umount_mp(const char *mp)
{
    char *av[] = {"umount", (char *)mp, NULL};
    return run_child(av);
}

/**
 * @brief Convert extracted binary to JSON and publish via MQTT
 *
 * This function will convert the binary file to a JSON structure and publish
 * via MQTT to a specified broker in an asynchronous way.
 *
 * @param folder Path to the extracted binary file
 */
static void convert_and_publish(const char *folder)
{
    /* Locate the binary file */
    char binpath[PATH_MAX];
    if (path_join(folder, BIN_NAME, binpath) < 0)
    {
        fprintf(stderr, "path too long: \"%s/%s\"\n", folder, BIN_NAME);
        return;
    }
    FILE *fp = fopen(binpath, "rb");
    if (!fp)
    {
        perror(binpath);
        return;
    }

    /* Initialise MQTT */
    mosquitto_lib_init();
    struct mosquitto *m = mosquitto_new(NULL, true, NULL);
    if (!m || mosquitto_connect_async(m, BROKER_ADDR, BROKER_PORT, 60))
    {
        fprintf(stderr, "MQTT init failed\n");
        fclose(fp);
        return;
    }
    mosquitto_loop_start(m);

    uint8_t buf[RECORD_SIZE];
    uint32_t tms;
    int16_t v[6];
    int published = 0;
    while (fread(buf, 1, RECORD_SIZE, fp) == RECORD_SIZE)
    {
        memcpy(&tms, buf, 4);
        for (int i = 0; i < 6; i++)
        {
            memcpy(&v[i], buf + 4 + i * 2, 2);
        }
        char js[512];
        int len = snprintf(js, sizeof js,
                           "{\"timestamp_ms\":%u,"
                           "\"acceleration\":[%.2f,%.2f,%.2f],"
                           "\"gyroscope\":[%.2f,%.2f,%.2f]}",
                           tms,
                           v[0] / SCALE, v[1] / SCALE, v[2] / SCALE,
                           v[3] / SCALE, v[4] / SCALE, v[5] / SCALE);
        mosquitto_publish(m, NULL, MQTT_TOPIC, len, js, 0, false);
        usleep(1000);
        published++;
    }

    mosquitto_loop_stop(m, true);
    mosquitto_destroy(m);
    mosquitto_lib_cleanup();
    fclose(fp);

    printf("Published %d IMU records from %s\n", published, folder);

    /* archive folder */
    mkdir(ARCHIVE_DIR, 0755);
    char target[PATH_MAX];
    const char *name = strrchr(folder, '/');
    if (!name)
    {
        name = folder;
    }
    else
    {
        name++;
    }

    if (path_join(ARCHIVE_DIR, name, target) < 0)
    {
        fprintf(stderr, "archive path too long: \"%s/%s\"\n", ARCHIVE_DIR, name);
    }
    else
    {
        if (rename(folder, target) < 0)
        {
            perror("rename to archive");
        }
    }
}

static void wait_for_clean_mountpoint(void)
{
    if (lfs_pid > 0)
    {
        int status;
        for (int i = 0; i < 50 && waitpid(lfs_pid, &status, WNOHANG) == 0; i++)
        {
            usleep(100000); 
        }
        lfs_pid = -1; 
    }

    for (int i = 0; i < 50 && is_fuse_mounted(MOUNT_POINT); i++)
    {
        usleep(100000); 
    }
}

/**
 * @brief Integrate all operations regarding to the wearable device
 *
 * This function will perform DFU if new firmware available, extract binary file from external flash
 * and publish them to the specified broker via MQTT.
 *
 * @param udev USB device
 *
 */
static void handle_device(struct udev *udev)
{
    /* 1) DFU upgrade if any */
    char *fw = next_firmware();
    if (fw)
    {

        puts("-> Firmware found, running DFU...");

        perform_dfu(fw);

        free(fw);
    }

    /* 2) find block device */
    char *devnode = NULL;
    for (int i = 0; i < 30 && !devnode; i++)
    {
        devnode = find_block_dev(udev, 2000);
    }
        
    if (!devnode)
    {
        fprintf(stderr, "!! no block dev\n");
        return;
    }

    /* 3) prepare destination */
    time_t tt = time(NULL);
    struct tm tm;
    localtime_r(&tt, &tm);
    char dest[PATH_MAX];
    strftime(dest, sizeof dest, DEST_BASE "/%Y%m%d_%H%M%S", &tm);
    mkdir(DEST_BASE, 0755);
    mkdir(dest, 0755);

    struct stat st;
    if (lstat(MOUNT_POINT, &st) == 0)
    {
        if (!S_ISDIR(st.st_mode))
        {
            unlink(MOUNT_POINT);
            mkdir(MOUNT_POINT, 0755); 
        }
    }
    else
    {
        mkdir(MOUNT_POINT, 0755); 
    }

    wait_for_clean_mountpoint(); 

    /* 4) mount & copy */
    lfs_pid = start_lfs(devnode);

    if (wait_for_file(MOUNT_POINT, BIN_NAME, 5000) != 0)
    {
        fprintf(stderr, "imu_log.bin did not appear within 5s - aborting copy\n");
        umount_mp(MOUNT_POINT);
        wait_for_clean_mountpoint(); 
        free(devnode);
        return;
    }

    if (copy_tree(MOUNT_POINT, dest) != 0)
    {
        fprintf(stderr, " X copy error\n"); 
    }

    if (is_fuse_mounted(MOUNT_POINT))
    {
        umount_mp(MOUNT_POINT);
        wait_for_clean_mountpoint(); 
    }

    free(devnode);

    /* 5) convert and publish then archive */
    convert_and_publish(dest);
}

int main(void)
{
    signal(SIGCHLD, reap);
    signal(SIGINT, forward_quit);
    signal(SIGTERM, forward_quit);

    struct udev *udev = udev_new();
    struct udev_monitor *mon = udev_monitor_new_from_netlink(udev, "udev");
    udev_monitor_filter_add_match_subsystem_devtype(mon, "usb", "usb_device");
    udev_monitor_enable_receiving(mon);
    int fd = udev_monitor_get_fd(mon);

    printf("Waiting for USB %s:%s ...\n", VENDOR_ID, PRODUCT_ID);

    while (!quit_flag)
    {
        /* debounce removal */
        if (debouncing && remove_seen_at_ms > 0 && now_ms() - remove_seen_at_ms > 500)
        {
            debouncing = 0;
            current_usb_path[0] = 0;
            puts("Device removed, idle");
        }

        struct pollfd pfd = {fd, POLLIN, 0};
        if (poll(&pfd, 1, 1000) <= 0)
        {
            continue;
        }
        
        struct udev_device *dev = udev_monitor_receive_device(mon);
        if (!dev)
        {
            continue;
        }
    
        const char *act = udev_device_get_action(dev),
                   *vid = udev_device_get_sysattr_value(dev, "idVendor"),
                   *pid = udev_device_get_sysattr_value(dev, "idProduct"),
                   *spath = udev_device_get_syspath(dev);

        if (!debouncing && act && vid && pid && !strcmp(act, "add") && !strcmp(vid, VENDOR_ID) && !strcmp(pid, PRODUCT_ID))
        {
            puts("Wearable detected - processing");
            handle_device(udev);
            puts("Waiting for removal ...");
            strncpy(current_usb_path, spath, sizeof current_usb_path - 1);
            debouncing = 1;
            remove_seen_at_ms = 0;
        }
        else if (debouncing && act && !strcmp(act, "remove") && !strcmp(spath, current_usb_path))
        {
            remove_seen_at_ms = now_ms();
        }

        udev_device_unref(dev);
    }

    puts("Shutdown requested");
    return 0;
}

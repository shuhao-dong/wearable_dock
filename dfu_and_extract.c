/*
 * wearable_auto_extract.c – DFU + LittleFS extractor (24 / 7 edition)
 *
 * Build : gcc -std=c11 -Wall -O2 dfu_and_extract.c  -ludev -o ~/dfu_and_extract
 * Needs : libudev-dev, FUSE, littlefs-fuse (lfs), dfu-util ≥ 0.11
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

#ifndef PATH_MAX
#define PATH_MAX 4096
#endif
#ifndef FUSE_SUPER_MAGIC
#define FUSE_SUPER_MAGIC 0x65735546
#endif

/* ─────── Runtime configuration ──────────────────────────────────── */
#define VENDOR_ID "0001"
#define PRODUCT_ID "0001"

#define LFS_BIN "/home/torus-pi5/littlefs-fuse/lfs"
#define LFS_ARGS "--block_count=1760 --block_size=4096 --read_size=16 " \
                 "--prog_size=16 --cache_size=64 --lookahead_size=32"

#define MOUNT_POINT "/mnt/wearable"
#define DEST_BASE "/home/torus-pi5/wearable_dock/extracted"

#define DFU_UTIL "/usr/bin/dfu-util"
#define FW_DIR "/home/torus-pi5/wearable_dock/new_firmware" /* expects *.bin */
#define FW_ARCHIVE "/home/torus-pi5/wearable_dock/new_firmware/archive"

/* ─────────────────────────────────────────────────────────────────── */

static int debouncing = 0;         /* 1 = we already processed plug-in */
static long remove_seen_at_ms = 0; /* time-stamp of last matching “remove” */

static char current_usb_path[PATH_MAX] = "";

static char src_root[PATH_MAX], dst_root[PATH_MAX];
static volatile sig_atomic_t lfs_pid = -1;

/* ───── Helpers: run program & wait ──────────────────────────────── */
static long now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long)ts.tv_sec * 1000 + ts.tv_nsec / 1e6;
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
    waitpid(pid, &st, 0);
    return (WIFEXITED(st) ? WEXITSTATUS(st) : -1);
}

/* ---------------------------------------------------------------
 * Delete everything under SRC (but not SRC itself).
 * ------------------------------------------------------------- */
static int rm_cb(const char *fpath, const struct stat *sb,
                 int typeflag, struct FTW *ftwbuf)
{
    (void)sb;

    if (ftwbuf->level == 0) /* this is SRC itself → skip */
        return 0;

    if (typeflag == FTW_F || typeflag == FTW_SL)
    {
        return remove(fpath); /* unlink file or symlink */
    }
    if (typeflag == FTW_DP)
    { /* directory after children */
        return rmdir(fpath);
    }
    return 0;
}

static int clear_tree(const char *src)
{
    /* FTW_DEPTH = post-order; FTW_PHYS = don’t follow symlinks */
    return nftw(src, rm_cb, 16, FTW_DEPTH | FTW_PHYS);
}

/* ───── fast file copy (256 KiB) ─────────────────────────────────── */
static int copy_file(int in, int out)
{
    const size_t B = 256 * 1024;
    char *buf = malloc(B);
    if (!buf)
    {
        return -1;
    }

    ssize_t r;
    while ((r = read(in, buf, B)) > 0)
        if (write(out, buf, r) != r)
        {
            r = -1;
            break;
        }
    free(buf);
    return (r < 0) ? -1 : 0;
}

/* nftw callback */
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
    if (flag == FTW_F)
    {
        int in = open(f, O_RDONLY), out = open(dst, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (in < 0 || out < 0)
        {
            return perror("open"), -1;
        }

        int rc = copy_file(in, out);
        close(in);
        close(out);
        return rc;
    }
    return 0;
}

static int copy_tree(const char *src, const char *dst)
{
    snprintf(src_root, sizeof src_root, "%s", src);
    snprintf(dst_root, sizeof dst_root, "%s", dst);
    return nftw(src, cp_cb, 16, FTW_PHYS);
}

/* ───── signal handlers ──────────────────────────────────────────── */
static void reap(int sig)
{
    (void)sig;
    while (waitpid(-1, NULL, WNOHANG) > 0)
        ;
}

static volatile sig_atomic_t quit_flag = 0;

static void forward_quit(int sig)
{
    if (lfs_pid > 0)
    {
        kill(lfs_pid, sig);
    }

    quit_flag = sig;
}

/* ───── udev: locate /dev/sdX for our VID/PID ─────────────────────── */
static char *find_block_dev(struct udev *u, int timeout_ms)
{
    long deadline = (long)(time(NULL) * 1000) + timeout_ms;
    while ((long)(time(NULL) * 1000) < deadline)
    {
        struct udev_enumerate *en = udev_enumerate_new(u);
        udev_enumerate_add_match_subsystem(en, "block");
        udev_enumerate_add_match_property(en, "DEVTYPE", "disk");
        udev_enumerate_scan_devices(en);
        struct udev_list_entry *e, *list = udev_enumerate_get_list_entry(en);
        udev_list_entry_foreach(e, list)
        {
            struct udev_device *blk =
                udev_device_new_from_syspath(u, udev_list_entry_get_name(e));
            struct udev_device *usb =
                udev_device_get_parent_with_subsystem_devtype(blk, "usb", "usb_device");
            const char *vid = usb ? udev_device_get_sysattr_value(usb, "idVendor") : NULL;
            const char *pid = usb ? udev_device_get_sysattr_value(usb, "idProduct") : NULL;
            if (vid && pid && !strcmp(vid, VENDOR_ID) && !strcmp(pid, PRODUCT_ID))
            {
                const char *node = udev_device_get_devnode(blk);
                if (node)
                {
                    char *ret = strdup(node);
                    udev_enumerate_unref(en);
                    return ret;
                }
            }
            udev_device_unref(blk);
        }
        udev_enumerate_unref(en);
        usleep(250 * 1000);
    }
    return NULL;
}

/* ───── DFU helpers ───────────────────────────────────────────────── */
/* find first *.bin in FW_DIR -> returns malloc'ed path or NULL */
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
/* extract serial string from `dfu-util -l` */
static int get_dfu_serial(char *ser, size_t len)
{
    int pipefd[2];
    if (pipe(pipefd))
    {
        return -1;
    }

    pid_t pid = fork();
    if (pid < 0)
    {
        return -1;
    }

    if (pid == 0)
    {
        dup2(pipefd[1], STDOUT_FILENO);
        close(pipefd[0]);
        char *av[] = {(char *)DFU_UTIL, "-l", NULL};
        execvp(av[0], av);
    }
    close(pipefd[1]);
    FILE *fp = fdopen(pipefd[0], "r");
    if (!fp)
    {
        return -1;
    }

    char line[512];
    int ok = -1;
    while (fgets(line, sizeof line, fp))
    {
        if (strstr(line, VENDOR_ID) && strstr(line, PRODUCT_ID) && strstr(line, "serial="))
        {
            char *s = strstr(line, "serial=");
            s += 7;
            strtok(s, " \n\r");
            strncpy(ser, s, len - 1);
            ser[len - 1] = '\0';
            ok = 0;
            break;
        }
    }
    fclose(fp);
    waitpid(pid, NULL, 0);
    return ok;
}
/* run dfu-util commands */
static int perform_dfu(const char *serial, const char *bin)
{
    char sn_arg[64];
    snprintf(sn_arg, sizeof sn_arg, "-s"); // same flag reused
    /* step 1: -e (detach) */
    char *av1[] = {(char *)DFU_UTIL, "-s", (char *)serial, "-e", NULL};
    if (run_child(av1) != 0)
    {
        fprintf(stderr, "DFU detach failed\n");
        return -1;
    }

    /* short wait for re-enumeration */
    sleep(2);

    /* step 2: download */
    char *av2[] = {(char *)DFU_UTIL, "-a", "1", "-D", (char *)bin, NULL};
    if (run_child(av2) != 0)
    {
        fprintf(stderr, "DFU download failed\n");
        return -1;
    }

    /* success → archive / delete firmware so we don't flash again */
    mkdir(FW_ARCHIVE, 0755);

    char archived[PATH_MAX];
    time_t tt = time(NULL);
    struct tm tm;
    localtime_r(&tt, &tm);
    strftime(archived, sizeof archived,
             FW_ARCHIVE "/%Y%m%d_%H%M%S.bin", &tm);

    if (rename(bin, archived) != 0)
    { /* fallback: unlink on failure */
        perror("rename");
        unlink(bin);
    }
    return 0;
}

/* ───── mount LittleFS (foreground) ───────────────────────────────── */
static pid_t start_lfs(const char *dev)
{
    pid_t pid = fork();
    if (pid < 0)
    {
        return -1;
    }

    if (pid == 0)
    {
        char *dup = strdup(LFS_ARGS);
        char *argv[32];
        int idx = 0;
        argv[idx++] = (char *)LFS_BIN;
        argv[idx++] = "-f";
        for (char *t = strtok(dup, " "); t && idx < 30; t = strtok(NULL, " "))
            argv[idx++] = t;
        argv[idx++] = (char *)dev;
        argv[idx++] = (char *)MOUNT_POINT;
        argv[idx] = NULL;
        execv(argv[0], argv);
        _exit(127);
    }
    return pid;
}
/* check if mp is a live FUSE mount */
static int is_fuse_mounted(const char *mp)
{
    struct statfs s;
    return statfs(mp, &s) == 0 && s.f_type == FUSE_SUPER_MAGIC;
}
/* simple umount */
static int umount_mp(const char *mp)
{
    char *av[] = {"umount", (char *)mp, NULL};
    return run_child(av);
}

/* ───── single plug-event workflow ───────────────────────────────── */
static void handle_device(struct udev *udev)
{
    /* --- 1. optional DFU upgrade --------------------------------- */
    char *fw = next_firmware();
    if (fw)
    {
        char serial[128];
        if (get_dfu_serial(serial, sizeof serial) == 0)
        {
            puts("  -> Firmware found, starting DFU ...");
            if (perform_dfu(serial, fw) == 0)
            {
                puts("  -> DFU OK, waiting for reboot ...");
            }
            else
            {
                fputs("  !! DFU failed - skipping extraction\n", stderr);
            }
        }
        else
            fputs("  !! Can't get DFU serial - skipping DFU\n", stderr);
        free(fw);
        /* wait up to 15 s for storage to come back */
    }

    char *devnode = NULL;
    for (int i = 0; i < 30 && !devnode; i++)
    {
        devnode = find_block_dev(udev, 500);
    }

    if (!devnode)
    {
        fputs("  !! No block device after DFU - abort\n", stderr);
        return;
    }
    printf("  -> Using device %s\n", devnode);

    /* --- 2. extraction ------------------------------------------- */
    time_t t = time(NULL);
    struct tm tm;
    localtime_r(&t, &tm);
    char dest[PATH_MAX];
    strftime(dest, sizeof dest, DEST_BASE "/%Y%m%d_%H%M%S", &tm);
    mkdir(DEST_BASE, 0755);
    mkdir(dest, 0755);
    mkdir(MOUNT_POINT, 0755);

    lfs_pid = start_lfs(devnode);
    if (lfs_pid < 0)
    {
        free(devnode);
        return;
    }
    sleep(1);

    printf("  -> Copying files → %s ...\n", dest);
    if (copy_tree(MOUNT_POINT, dest) == 0)
    {
        puts("  Y Extraction complete - deleting source files …");
        if (clear_tree(MOUNT_POINT) == 0)
            puts("  Y Source flash wiped");
        else
            fputs("  !! Couldn't wipe flash - continuing\n", stderr);
    }
    else
    {
        fputs("  X Copy error - leaving flash untouched\n", stderr);
    }

    if (is_fuse_mounted(MOUNT_POINT))
        umount_mp(MOUNT_POINT);
    waitpid(lfs_pid, NULL, 0);
    lfs_pid = -1;
    free(devnode);
}

/* ───── main loop ────────────────────────────────────────────────── */
int main(void)
{
    signal(SIGCHLD, reap);
    signal(SIGINT, forward_quit);
    signal(SIGTERM, forward_quit);

    struct udev *udev = udev_new();
    if (!udev)
    {
        perror("udev");
        return 1;
    }
    struct udev_monitor *mon = udev_monitor_new_from_netlink(udev, "udev");
    udev_monitor_filter_add_match_subsystem_devtype(mon, "usb", "usb_device");
    udev_monitor_enable_receiving(mon);
    int fd = udev_monitor_get_fd(mon);

    printf("Waiting for USB %s:%s ...\n", VENDOR_ID, PRODUCT_ID);

    while (!quit_flag)
    {
        /* -------- 1. timeout: has the device been gone ≥ 500 ms ? ---- */
        if (debouncing && remove_seen_at_ms > 0 &&
            now_ms() - remove_seen_at_ms > 500) /* 0.5 s */
        {
            debouncing = 0; /* ready for next plug-in */
            current_usb_path[0] = '\0';
            remove_seen_at_ms = 0;
            puts("Device removed - back to idle.");
        }

        /* -------- 2. wait for the next udev event (up to 1 s) -------- */
        struct pollfd pfd = {fd, POLLIN, 0};
        int ret = poll(&pfd, 1, 1000); /* 1-s tick / Ctrl-C */
        if (ret <= 0)
        {
            continue; /* timeout or EINTR → loop again */
        }

        struct udev_device *dev = udev_monitor_receive_device(mon);
        if (!dev)
        {
            continue; /* defensive */
        }
            
        const char *action = udev_device_get_action(dev); /* "add"/"remove" */
        const char *vid = udev_device_get_sysattr_value(dev, "idVendor");
        const char *pid = udev_device_get_sysattr_value(dev, "idProduct");
        const char *spath = udev_device_get_syspath(dev); /* usb_device path */

        /* ---------- first “add” after a real plug-in ----------------- */
        if (!debouncing &&
            action && strcmp(action, "add") == 0 &&
            vid && pid &&
            strcmp(vid, VENDOR_ID) == 0 && strcmp(pid, PRODUCT_ID) == 0)
        {
            puts("Wearable detected - processing ...");
            handle_device(udev); /* DFU + extraction */
            puts("Waiting for stable removal ...");

            strncpy(current_usb_path, spath,
                    sizeof current_usb_path - 1);
            debouncing = 1;        /* block duplicates */
            remove_seen_at_ms = 0; /* reset timer      */
        }
        /* ---------- any “remove” of *that* usb_device ---------------- */
        else if (debouncing &&
                 action && strcmp(action, "remove") == 0 &&
                 strcmp(spath, current_usb_path) == 0)
        {
            remove_seen_at_ms = now_ms(); /* start vanish timer */
        }

        udev_device_unref(dev); /* always drop ref */
    }

    puts("Shutdown requested");
    return 0;
}

/*
 * imu2json_mqtt.c
 *
 *  Read the latest imu_log.bin from
 *  /home/torus-pi5/wearable_dock/extracted/<timestamp>
 *  convert its records to JSON, and publish to MQTT.
 *  Then move that entire <timestamp> folder into
 *  /home/torus-pi5/wearable_dock/extracted/archive.
 *
 *  Compile:
 *    cc -Wall -O2 bin2json_mqtt.c -o ~/bin2json_mqtt -lmosquitto
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <mosquitto.h>
#include <unistd.h>
#include <sys/stat.h>
#include <errno.h>

#define EXTRACTED_BASE  "/home/torus-pi5/wearable_dock/extracted"
#define ARCHIVE_SUBDIR  "archive"
#define BIN_NAME        "imu_log.bin"

#define BROKER_ADDR   "localhost"
#define BROKER_PORT   1883
#define MQTT_TOPIC    "BORUS/extf"

#define RECORD_SIZE   (4 + 6*2)   /* 4-byte uint32 + 6x2-byte int16 */
#define SCALE_FACTOR  100.0f

#ifndef PATH_MAX
# define PATH_MAX 4096
#endif

/* Find the lexicographically greatest subdirectory (timestamped). */
static char *find_latest_folder(void) {
    DIR *d = opendir(EXTRACTED_BASE);
    if (!d) { perror("opendir"); return NULL; }
    struct dirent *e;
    char *best = NULL;
    while ((e = readdir(d))) {
        if (e->d_type != DT_DIR) continue;
        if (strcmp(e->d_name, ".")==0 || strcmp(e->d_name,"..")==0) continue;
        if (strcmp(e->d_name, ARCHIVE_SUBDIR)==0) continue;
        if (!best || strcmp(e->d_name, best) > 0) {
            free(best);
            best = strdup(e->d_name);
        }
    }
    closedir(d);
    return best;  /* caller must free */
}

/* Safely join two path components with a slash into out[], checking PATH_MAX. */
static int path_join(const char *a, const char *b, char *out) {
    size_t la = strlen(a), lb = strlen(b);
    if (la + 1 + lb >= PATH_MAX) {
        fprintf(stderr, "path too long: \"%s/%s\"\n", a, b);
        return -1;
    }
    memcpy(out, a, la);
    out[la] = '/';
    memcpy(out+la+1, b, lb);
    out[la+1+lb] = '\0';
    return 0;
}

int main(void)
{
    /* 1) locate the latest timestamped folder */
    char *ts = find_latest_folder();
    if (!ts) {
        fprintf(stderr, "No timestamped folders found in %s\n", EXTRACTED_BASE);
        return 1;
    }

    /* build & check paths */
    char folder_path[PATH_MAX], bin_path[PATH_MAX];
    if (path_join(EXTRACTED_BASE, ts, folder_path) != 0) {
        free(ts); return 1;
    }
    if (path_join(folder_path, BIN_NAME, bin_path) != 0) {
        free(ts); return 1;
    }

    /* 2) open the bin */
    FILE *fp = fopen(bin_path, "rb");
    if (!fp) {
        perror(bin_path);
        free(ts);
        return 1;
    }

    /* 3) init MQTT */
    mosquitto_lib_init();
    struct mosquitto *m = mosquitto_new(NULL, true, NULL);
    if (!m || mosquitto_connect_async(m, BROKER_ADDR, BROKER_PORT, 60)) {
        fprintf(stderr, "MQTT init/connect failed\n");
        fclose(fp); free(ts);
        return 1;
    }
    mosquitto_loop_start(m);

    /* 4) read, convert & publish */
    uint8_t buf[RECORD_SIZE];
    uint32_t ts_ms;
    int16_t v[6];
    while (fread(buf,1,RECORD_SIZE,fp) == RECORD_SIZE) {
        memcpy(&ts_ms, buf, 4);
        for (int i = 0; i < 6; i++) {
            int16_t x;
            memcpy(&x, buf + 4 + i*2, 2);
            v[i] = x;
        }
        char js[256];
        int len = snprintf(js, sizeof js,
            "{"
              "\"timestamp_ms\":%u,"
              "\"acceleration\":[%.2f,%.2f,%.2f],"
              "\"gyroscope\":[%.2f,%.2f,%.2f]"
            "}",
            ts_ms,
            v[0]/SCALE_FACTOR, v[1]/SCALE_FACTOR, v[2]/SCALE_FACTOR,
            v[3]/SCALE_FACTOR, v[4]/SCALE_FACTOR, v[5]/SCALE_FACTOR
        );
        fputs(js, stdout);
        fflush(stdout);
        mosquitto_publish(m, NULL, MQTT_TOPIC, len, js, 0, false);
        usleep(1000);
    }

    /* 5) cleanup MQTT & file */
    mosquitto_loop_stop(m, true);
    mosquitto_destroy(m);
    mosquitto_lib_cleanup();
    fclose(fp);

    /* 6) move the entire folder into ?archive? */
    char archive_dir[PATH_MAX], target_path[PATH_MAX];
    if (path_join(EXTRACTED_BASE, ARCHIVE_SUBDIR, archive_dir) != 0 ||
        (mkdir(archive_dir, 0755) < 0 && errno!=EEXIST) ||
        path_join(archive_dir, ts, target_path) != 0)
    {
        perror("prepare archive");
        free(ts);
        return 1;
    }
    if (rename(folder_path, target_path) != 0) {
        perror("rename to archive");
        /* not fatal */
    }

    free(ts);
    return 0;
}

/*
 * imu2json_mqtt.c
 *
 *  Read imu_log.bin, convert its records to JSON, and publish to MQTT.
 *
 *  Compile:
 *    cc -Wall -O2 bin2json_mqtt.c -o bin2json_mqtt -lmosquitto
 *
 *  Usage:
 *    ./imu2json_mqtt <path/to/imu_log.bin>
 *
 *  Publishes each record as:
 *    {
 *      "timestamp_ms": 1234,
 *      "acceleration":[x,y,z],
 *      "gyroscope":[x,y,z]
 *    }
 *  to MQTT topic BORUS/extf on localhost:1883.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <mosquitto.h>
#include <unistd.h>

#define BROKER_ADDR   "localhost"
#define BROKER_PORT   1883
#define MQTT_TOPIC    "BORUS/extf"

#define RECORD_FMT    "<Ihhhhhh"
#define RECORD_SIZE   (4 + 6*2)   /* 4-byte uint32 + 6x2-byte int16 */
#define SCALE_FACTOR  100.0f

int main(int argc, char **argv)
{
    if(argc!=2){
        fprintf(stderr,"Usage: %s <imu_log.bin>\n",argv[0]);
        return 1;
    }

    const char *infile = argv[1];
    FILE *fp = fopen(infile,"rb");
    if(!fp){
        perror("fopen");
        return 1;
    }

    /* --- init MQTT --- */
    mosquitto_lib_init();
    struct mosquitto *m = mosquitto_new(NULL,true,NULL);
    if(!m){
        fprintf(stderr,"Error: mosquitto_new\n");
        return 1;
    }
    if(mosquitto_connect_async(m,BROKER_ADDR,BROKER_PORT,60)){
        fprintf(stderr,"Error: mosquitto_connect\n");
        return 1;
    }
    mosquitto_loop_start(m);

    /* --- read & publish --- */
    uint8_t buf[RECORD_SIZE];
    uint32_t ts;
    int16_t v[6];

    while(fread(buf,1,RECORD_SIZE,fp)==RECORD_SIZE){
        /* unpack */
        memcpy(&ts,buf,4);
        for(int i=0;i<6;i++){
            int16_t x;
            memcpy(&x, buf+4 + i*2, 2);
            v[i]=x;
        }

        /* build JSON */
        char js[256];
        int len = snprintf(js,sizeof(js),
            "{"
              "\"timestamp_ms\":%u,"
              "\"acceleration\":[%.2f,%.2f,%.2f],"
              "\"gyroscope\":[%.2f,%.2f,%.2f]"
            "}",
            ts,
            v[0]/SCALE_FACTOR, v[1]/SCALE_FACTOR, v[2]/SCALE_FACTOR,
            v[3]/SCALE_FACTOR, v[4]/SCALE_FACTOR, v[5]/SCALE_FACTOR
        );

        fputs(js, stdout);
        fflush(stdout);

        /* publish */
        mosquitto_publish(m,NULL,MQTT_TOPIC,len,js,0,false);
        /* tiny delay to avoid flooding */
        usleep(1000);
    }

    /* --- clean up --- */
    mosquitto_loop_stop(m,true);
    mosquitto_destroy(m);
    mosquitto_lib_cleanup();
    fclose(fp);
    return 0;
}

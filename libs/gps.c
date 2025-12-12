// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include Pico libraries
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/spi.h"
#include "hardware/adc.h"
#include "hardware/uart.h"

#include "vga16_graphics_v2.h"
#include "vga_graphics_v3.h"



char read_buff[82];
char nmea_gpgga[82];

float lat;
float lon;
float utc;

void parseNMEA(char raw_nmea[]) {
    char* nmea_sentence = strstr((char*)raw_nmea, "$GPGGA");
    
    if (nmea_sentence != NULL) {
        for (int i = 1; i < strlen(nmea_sentence); i++) {
            if (nmea_sentence[i] == '$') {
                strncpy(nmea_gpgga, nmea_sentence, 82);
                break;
            }
        }
    }
}

void getLat() { 
    uint8_t comma_count = 0; 
    char lat_str[12];
    for (int i = 0; i < 82; i++) {
        if (nmea_gpgga[i] == ',') {
            comma_count++;
        }
        if (comma_count == 4) {
            printf("%s\n\n", strncpy(lat_str, nmea_gpgga, i));
        }
    }
    
    return;
}

float getLon() {
    return lon;
}

void getUTC() {
    uint8_t comma_count = 0; 
    char utc_str[11];
    for (int i = 0; i < 82; i++) {
        if (nmea_gpgga[i] == ',') {
            comma_count++;
        }
        if (comma_count == 2) {
            printf("%s\n\n", strncpy(utc_str, nmea_gpgga + 7, i - 7));
        }
    }

    return; 
}
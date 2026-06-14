#include <stdio.h>
#include <stdlib.h>
#include "sensor_telemetry.h"

int main(int argc, char *argv[]) {
    const char *path = (argc > 1) ? argv[1] : "/ariss/sensors.rt";

    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        perror("fopen");
        return 1;
    }

    sensor_telemetry_t t;
    size_t n = fread(&t, 1, sizeof(t), f);
    fclose(f);

    if (n != sizeof(t)) {
        fprintf(stderr, "Short read: got %zu bytes, expected %zu\n",
                n, sizeof(t));
        return 1;
    }

    printf("timestamp        %u\n", t.timestamp);
    printf("LPS25_pressure   %u   (%.2f hPa)\n", t.LPS25_pressure, t.LPS25_pressure / 4096.0);
    printf("LPS25_temp       %u   (%.2f C)\n", (short)t.LPS25_temp, (short)t.LPS25_temp / 100.0);
    printf("HTS221_temp      %u   (%.2f C)\n", (short)t.HTS221_temp, (short)t.HTS221_temp / 100.0);
    printf("HTS221_humidity  %u   (%.2f %%)\n", t.HTS221_humidity, t.HTS221_humidity / 100.0);
    printf("AccelerationX    %d\n", (short)t.AccelerationX);
    printf("AccelerationY    %d\n", (short)t.AccelerationY);
    printf("AccelerationZ    %d\n", (short)t.AccelerationZ);
    printf("GyroX            %d\n", (short)t.GyroX);
    printf("GyroY            %d\n", (short)t.GyroY);
    printf("GyroZ            %d\n", (short)t.GyroZ);
    printf("MagX             %d\n", (short)t.MagX);
    printf("MagY             %d\n", (short)t.MagY);
    printf("MagZ             %d\n", (short)t.MagZ);
    printf("IMUTemp          %d\n", (short)t.IMUTemp);
    printf("light_level      %u\n", t.light_level);
    printf("light_RGB        0x%06X\n", t.light_RGB);
    printf("ImuValid         %u\n", t.ImuValid);
    printf("TempHumidityValid %u\n", t.TempHumidityValid);
    printf("PressureValid    %u\n", t.PressureValid);
    printf("ColorValid       %u\n", t.ColorValid);

    return 0;
}


Note: based off https://github.com/wejn/esp32-huello-world/tree/master for functionning ha binding

| Supported Targets | ESP32-H2 | ESP32-C6 |
| ----------------- | -------- | -------- |

# Light Bulb Example 

This test code shows how to configure Zigbee end device and use it as HA on/off light bulb

## Hardware Required

* One development board with ESP32-H2 SoC acting as Zigbee end-device (loaded with HA_on_off_light example)
* A USB cable for power supply and programming
* Choose another ESP32-H2 as Zigbee coordinator (see [HA_on_off_switch example](../HA_on_off_switch/))

## Configure the project

Before project configuration and build, make sure to set the correct chip target using `idf.py set-target TARGET` command.

idf.py build -DIDF_TARGET=esp32c6 -DESP_ZB_SDK_DIR:PATH=E:\dev\embedded\esp-zigbee-sdk

idf.py -p COM3 flash

python esp_zb_mfg_tool.py -m CAFEBEEF50C0FFE0 -c 0x07FFF800 -mn Nekinace -mc 0x131B

>remove the empty line from sigle_device.csv

python esp_zb_mfg_tool.py --csv sigle_device.csv

esptool.py -p COM3 write_flash 0x1d8000 bin/CAFEBEEF50C0FFE0.bin

idf.py -p COM3 monitor

## Erase the NVRAM 

Before flash it to the board, it is recommended to erase NVRAM if user doesn't want to keep the previous examples or other projects stored info 
using `idf.py -p PORT erase-flash`

## Build and Flash

Build the project, flash it to the board, and start the monitor tool to view the serial output by running `idf.py -p PORT flash monitor`.

(To exit the serial monitor, type ``Ctrl-]``.)

## Example Output

As you run the example, you will see the following log:

```
I (394) main_task: Calling app_main()
I (404) gpio: GPIO[8]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 1| Pulldown: 0| Intr:0 
I (404) phy_init: phy_version 220,2dbbbe7,Sep 25 2023,20:39:25
I (464) phy: libbtbb version: 90c587c, Sep 25 2023, 20:39:57
I (474) ESP_ZB_COLOR_DIMM_LIGHT: ZDO signal: ZDO Config Ready (0x17), status: ESP_FAIL
I (474) ESP_ZB_COLOR_DIMM_LIGHT: Zigbee stack initialized
I (484) ESP_ZB_COLOR_DIMM_LIGHT: Start network steering
I (484) main_task: Returned from app_main()
I (9614) ESP_ZB_COLOR_DIMM_LIGHT: ZDO signal: NWK Permit Join (0x36), status: ESP_OK
I (9834) ESP_ZB_COLOR_DIMM_LIGHT: ZDO signal: NWK Permit Join (0x36), status: ESP_OK
I (9834) ESP_ZB_COLOR_DIMM_LIGHT: Joined network successfully (Extended PAN ID: 60:55:f9:00:00:f6:07:b4, PAN ID: 0x2a74, Channel:13)
I (32944) ESP_ZB_COLOR_DIMM_LIGHT: Received message: endpoint(10), cluster(0x6), attribute(0x0), data size(1)
I (32944) ESP_ZB_COLOR_DIMM_LIGHT: Light sets to On
I (33984) ESP_ZB_COLOR_DIMM_LIGHT: Received message: endpoint(10), cluster(0x6), attribute(0x0), data size(1)
I (33984) ESP_ZB_COLOR_DIMM_LIGHT: Light sets to Off
I (35304) ESP_ZB_COLOR_DIMM_LIGHT: ZDO signal: NLME Status Indication (0x32), status: ESP_OK
I (35534) ESP_ZB_COLOR_DIMM_LIGHT: Received message: endpoint(10), cluster(0x6), attribute(0x0), data size(1)
I (35534) ESP_ZB_COLOR_DIMM_LIGHT: Light sets to On
```

## Light Control Functions

 * By toggling the switch button (BOOT) on the ESP32-H2 board loaded with the `HA_on_off_switch` example, the LED on this board loaded with `HA_on_off_light` example will be on and off.

## Troubleshooting

For any technical queries, please open an [issue](https://github.com/espressif/esp-zigbee-sdk/issues) on GitHub. We will get back to you soon.

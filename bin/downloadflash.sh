esptool.py --port /dev/ttyUSB0 write_flash 0x00000 eagle.flash.bin 0x20000 eagle.irom0text.bin 0x7E000 blank.bin 0x3fc000 esp_init_data_default.bin

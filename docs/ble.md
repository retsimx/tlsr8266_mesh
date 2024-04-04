# BLE OTA

The included BLE tools include a `flash_fw.py` script that can be used to flash new firmware to the TYBT1, it is compatible with the OEM firmware, as well as this firmware. 



Upgrading from the OEM firmware will require passing the `--force` flag, to ignore versions. Be *very* sure that the firmware you're flashing to the TYBT1 is a 12MHz firmware. Flashing a 16MHz firmware will brick the device (and it's virtually unrecoverable due to the way the TYBT1 is usually soldered to a larger board. The SWS pad is inaccessible typically).



The `meshutils/flash_fw.py` script requires the following parameters:

* `--mesh_name`, the mesh name that the light was paired with (for an unpaired light this is usually `out_of_mesh`)
* `--mesh_password`, the mesh password that the light was paired with (for an unpaired light this is usually `123`)
* `--fw_file`, the path to the firmware file to flash.
* `--mac`, The BLE MAC address of the device to flash the firmware to. eg `BC:23:4C:07:CE:CE`
* `--force`, an optional parameter that will bypass version checks and crystal speed checks. Use this with caution; Typically only use this for flashing this firmware over an OEM firmware.
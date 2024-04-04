# UART Updater

It is possible to use the UART updater once the E104-BT05-TB has been updated with this firmware. The UART updater can update both the TB and any node in the connected mesh. Note that the UART updater is slower than the BLE updater, but doesn't require you to be in range of the node you're trying to update. It's not uncommon for a firmware update to a node that is several hops away on the mesh to take 30 minutes or more in some cases.



The `meshutils/flash_fw_uart.py` script requires the following parameters:

* `--fw_file`, The firmware file to flash
* `--mesh_address`, The mesh node address to flash the firmware to (This is the address provided to the node when it was paired)
* `--force`, Optional parameter that will ignore the version check. Note that this does not bypass the crystal check - there should be no case where you want to bypass the crystal check once using this firmware.


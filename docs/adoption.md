# Mesh Adoption

All adoption works the same way for both the OEM firmware, and this firmware. Adoption logs in to the device with the default mesh credentials and then updates the mesh name and password, and configures a provided mesh address.



The mesh address must be unique (But does not need to be sequential), so make sure to track the addresses that you've assigned.



If starting a new mesh from scratch, you can generate your own mesh name and password. Both can be any sequence of printable characters, between 1 and 16 characters in length.



The included script `meshutils/mesh_add.py` takes the following arguments:

* `--mesh_address`, The mesh address to assign to the light. Currently we only support up to 63 lights in the mesh (0 is reserved). So this value should be a unique (in the mesh) number between 1 and 63.
* `--mesh_name`, The name of the BLE mesh
* `--mesh_password`, The password of the BLE mesh
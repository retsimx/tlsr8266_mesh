# Mesh Adoption

All adoption works the same way for both the OEM firmware, and this firmware. Adoption logs in to the device with the default mesh credentials and then updates the mesh name and password, and configures a provided mesh address.

## Mesh Addresses

The mesh address must be unique (but does not need to be sequential), so make sure to track the addresses that you've assigned. Currently, only addresses from 1 to 63 are supported, with address 0 reserved for special system functionality.

## Creating a New Mesh

If starting a new mesh from scratch, you can generate your own mesh name and password. Both can be any sequence of printable characters, between 1 and 16 characters in length.

For improved security, you should also generate a custom Long-Term Key (LTK). The LTK is a 16-byte (32 hex character) key used for encrypting mesh communications.

## Using the Mesh Addition Script

The included script `meshutils/mesh_add.py` is used to provision a device to join your mesh network. Before running the script:

1. Ensure the device is powered on and in an unpaired state (new devices or factory reset devices advertise with the name "out_of_mesh")
2. Make sure you have Python 3 and required dependencies installed (run `pip install -r utilities/meshutils/requirements.txt`)
3. Ensure you have Bluetooth permissions on your system

### Command Line Arguments

The script accepts the following arguments:

* `--mesh_address` (required): The mesh address to assign to the light. Currently we only support up to 63 lights in the mesh (0 is reserved). So this value should be a unique (in the mesh) number between 1 and 63.
* `--mesh_name` (required): The name of the BLE mesh. Must be 16 characters or fewer.
* `--mesh_password` (required): The password of the BLE mesh. Must be 16 characters or fewer.
* `--mesh_ltk` (optional): A custom Long-Term Key for the mesh in hex format (32 hex characters / 16 bytes). If not provided, the script will use a default LTK with a security warning and confirmation prompt.

### Usage Examples

```bash
# Add a light with address 5 to mesh "myhome" with password "securepass"
python mesh_add.py --mesh_address 5 --mesh_name myhome --mesh_password securepass

# Add a light with custom LTK (more secure)
python mesh_add.py --mesh_address 6 --mesh_name myhome --mesh_password securepass --mesh_ltk "a1b2c3d4e5f6a7b8c9d0e1f2a3b4c5d6"

# Add the first light to a new mesh network
python mesh_add.py --mesh_address 1 --mesh_name newmesh --mesh_password newpass
```

### Custom LTK for Security

Using a custom LTK is strongly recommended for production deployments as it enhances the security of your mesh network. You can generate a random LTK using standard tools:

```bash
# Using OpenSSL to generate a random 16-byte key in hex format
openssl rand -hex 16

# Or on Linux/Mac with dd and hexdump
dd if=/dev/urandom bs=16 count=1 2>/dev/null | hexdump -ve '1/1 "%.2x"'
```

### Pairing Process

The script performs the following steps:

1. **Device Discovery**: Scans for unpaired devices advertising with the default name
2. **Authentication**: Establishes a secure session with the device
3. **Address Assignment**: Configures the device with its unique mesh address
4. **Parameter Provisioning**: Sends the mesh name, password, and LTK to the device
5. **Verification**: Confirms that the device has successfully joined the mesh

If the pairing is successful, the device will join your mesh network and be controllable through mesh commands.

### Troubleshooting

If the script fails to discover or connect to the device:
- Ensure the device is in an unpaired state (factory reset if necessary)
- Check that Bluetooth is enabled on your system
- Verify you have the required permissions to use Bluetooth
- Try moving the device closer to your computer
- Run the script with verbose logging for debugging

If the script connects but fails during pairing:
- Check that your provided mesh parameters meet the length requirements
- Ensure the LTK (if provided) is in valid hex format
- Try the pairing process again from the beginning
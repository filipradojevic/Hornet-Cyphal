# FSMScriptForBlackBox

This script emulates a Ground Control Station (GCS) and sends MAVLink `COMMAND_LONG` messages over Ethernet to control the device FSM (Finite State Machine).

## FSM State Mapping
- **param2 = 4** — IDLE: no action
- **param2 = 1** — INITIALIZATION: initialize LittleFS and ULog (prepare logging)
- **param2 = 0** — OPERATIONAL: start regular logging
- **param2 = 2** — MAINTENANCE: enter File Transfer Protocol (FTP) mode
- **param2 = 3** — SOFTWARE_UPDATE: enter bootloader / software update mode

## Network Configuration
- Assign a static IP to the host machine (example): `192.168.1.151`.
- Configure the device / gateway IP to `192.168.1.150`.
- On Windows: Control Panel → Network Connections → [adapter] → Properties → IPv4 → Use the following IP address.

> Note: The script must bind to an IP address that actually exists on the host. If you encounter binding errors, either update the script to bind to `0.0.0.0` (all interfaces) or set the host adapter to the static IP above.

## Example
- Example invocation (adjust script name and options as needed):

```powershell
python send_command_long.py --target 192.168.1.150 --param2 2
```

## Tips
- Verify the correct network adapter is used (VPNs or virtual adapters can interfere).
- If you want a localized (Croatian) version, say so and I will add it.

---
Last updated: concise, professional README for quick reference.

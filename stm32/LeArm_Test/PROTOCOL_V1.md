# LeArm Serial Protocol V1

All multibyte values are little-endian. Every frame uses this layout:

```text
A5 5A | VERSION | SEQUENCE | COMMAND | PAYLOAD_LENGTH | PAYLOAD | CRC16_LO CRC16_HI
```

`VERSION` is `0x01`. CRC is CRC-16/CCITT-FALSE with initial value `0xFFFF`,
polynomial `0x1021`, no reflection, and covers bytes from `VERSION` through
the end of `PAYLOAD`.

Commands:

| Command | Payload | Response |
| --- | --- | --- |
| `0x10` move pulses | `duration_ms:u16`, `count:u8`, then `id:u8,pulse_us:u16` for each joint | `0x80` ACK |
| `0x11` get status | empty | `0x81` status |
| `0x12` emergency stop | empty | `0x80` ACK |
| `0x13` clear emergency stop | empty | `0x80` ACK |
| `0x14` get encoder | empty | `0x82` encoder status |

ACK payload is `request_command:u8,status:u8`. Status `0` means accepted.
The status payload is `flags:u8,moving_mask:u8,current_pulse_us[6],target_pulse_us[6]`.
Flag bit 0 means interpolation is active; bit 1 means software emergency stop
is latched. Emergency stop freezes each current PWM and holds servo torque
until a clear command is accepted.

The encoder status payload is `valid:u8,status:u8,raw_angle:u16` for the
single AS5600 sensor on the software I2C test bus. `valid` bit 0 means the
latest I2C read succeeded. `status` is the AS5600 status register, and
`raw_angle` is the 12-bit raw angle from registers `0x0C` and `0x0D`.

The protocol reports firmware targets, not physical joint feedback. Keep an
independent hardware emergency-stop or power cut-off for personnel safety.

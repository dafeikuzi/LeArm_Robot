# LeArm Serial Protocol V1

All multibyte values are little-endian. Every frame uses this layout:

```text
A5 5A | VERSION | SEQUENCE | COMMAND | PAYLOAD_LENGTH | PAYLOAD | CRC16_LO CRC16_HI
```

`VERSION` is `0x01`. CRC is CRC-16/CCITT-FALSE with initial value `0xFFFF`,
polynomial `0x1021`, no reflection, and covers bytes from `VERSION` through
the end of `PAYLOAD`.

The UART transport is `115200` baud, 8 data bits, no parity, and one stop bit
(8N1). TX and RX use 3.3 V TTL levels and must share a ground.

Commands:

| Command | Payload | Response |
| --- | --- | --- |
| `0x10` move pulses | `duration_ms:u16`, `count:u8`, then `id:u8,pulse_us:u16` for each joint | `0x80` ACK |
| `0x11` get status | empty | `0x81` status |
| `0x12` emergency stop | empty | `0x80` ACK |
| `0x13` clear emergency stop | empty | `0x80` ACK |

ACK payload is `request_command:u8,status:u8`. Status `0` means accepted.
The status payload is `flags:u8,moving_mask:u8,current_pulse_us[6],target_pulse_us[6]`.
Flag bit 0 means interpolation is active; bit 1 means software emergency stop
is latched. Emergency stop freezes each current PWM and holds servo torque
until a clear command is accepted.

For `move pulses`, `duration_ms` is a requested time. The firmware enforces a
minimum servo motion time of 1000 ms and may extend the actual interpolation
time further to satisfy per-servo PWM step limits. The servo interpolation uses
a quintic S-curve profile to reduce start/stop shock.

The firmware also emits a plain-text angle estimate line every 200 ms on the
same UART, for example `ANG J1=-27 J2=0 J3=0 J4=0 J5=0 J6=0 deg`. These are
PWM-derived estimates using `1500 us = 0 deg` and `500..2500 us = -90..90 deg`;
they are not physical feedback. The separate power-on pose is configured in
`Core/Src/learm_servo.c`.

The protocol reports firmware targets, not physical joint feedback. Keep an
independent hardware emergency-stop or power cut-off for personnel safety.

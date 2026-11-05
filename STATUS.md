# Pixel 8a (akita): status

HakiOS (iOS 16.5 on XNU) on the Google Pixel 8a. Last update: 2026-11-05.

## Works

| | |
|---|---|
| Boots | from the phone's storage (slot b), iOS up in about 40 s; Android stays on slot a |
| Screen and touch | yes (drawn in software for now, about 10 frames a second) |
| Buttons | power and volume |
| Battery | charge level and charging |
| Temperature and CPU speed | yes, all 9 cores |
| Clock | keeps the time |
| Shut down / restart | yes |
| USB | seen as an iPhone: pairing, iPhone tools, photos to a computer (Linux) |
| GPU | Mali-G715 runs, Metal copies, fills and compute programs on it |

## In progress

- **Framebuffer:** a faster screen than the software copy (about 10 frames a second).

## Not yet

- iOS drawing with the GPU (needs Metal rendering)
- Sound, Wi-Fi, Bluetooth, mobile network, camera, vibration, fingerprint, sensors
- Photos to Windows (needs Apple's own USB stack)

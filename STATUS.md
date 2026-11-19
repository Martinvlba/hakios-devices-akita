# Pixel 8a (akita): status

HakiOS (iOS 16.6.1 userland of the iPhone 13 mini, arm64e, on XNU; 16.7.16 of the iPhone X before 2026-11-16) on the
Google Pixel 8a (Tensor G3, Mali-G715). Last update: 2026-11-17.

The phone starts iOS from its own storage (slot b) in about 45 seconds; Android stays on slot a. Everything below is
what was seen on the one phone this port is developed on.

## Works

| | |
|---|---|
| Boot | from the phone's storage, straight to the lock or home screen; iOS's data is kept between starts |
| Safety net | a start that does not reach the home screen in 4 minutes restarts into the bootloader (fastboot) |
| Screen | 1080 x 2400 at 120 Hz, drawn by the GPU, applications' own Metal layers at the full 120 frames a second; brightness slider; screen off and on with the side button |
| Touch and buttons | touch, power, volume |
| GPU | Mali-G715 with a Metal driver of our own: the system's interface, the applications, Safari and WebGL draw with it (a 600-draw WebGL aquarium demo at the page's full 60 frames a second); Maps draws its map (land, water, roads, labels) |
| Bluetooth | on (on 16.6.1 too, since 2026-11-17): Settings' switch, discoverable as "iPhone", finds nearby devices; earbuds pair and play (YouTube in Safari heard through Nothing Ear (3)); calls' audio, keyboards and LE accessories not tried yet |
| Wi-Fi | joins WPA2 networks, DHCP, HTTPS; a network joined once in Settings is joined again by iOS itself after a restart (about 50 seconds after start); each phone has its own address |
| Sound | both speakers, with the iPhone 13 mini's own speaker processing (its speaker protection runs without live amplifier feedback for now) |
| Vibration | yes: the system's taps and vibrations, Core Haptics patterns |
| Flashlight | the LED works (Control Center's button not confirmed yet) |
| Battery | charge level and charging shown |
| Heat and speed | all 9 cores, speed scaled with load and temperature, cores idle when there is nothing to do |
| Clock | keeps the time |
| Shut down / restart | yes |
| Passcode | can be set, changed and removed in Settings; the phone locks and asks for it (kept in software, see below) |
| Auto-lock | the screen goes off and the phone locks after the time set in Settings |
| Stock applications | all installed, the removable ones (Mail, Maps, Notes, ...) included; every one was opened and draws its first screen (News loads its stories). Without their hardware or service: Magnifier and Measure are black and Camera has no picture (no camera yet), Compass does not turn (no sensor), Music says it is offline, Weather has no data |
| Screenshots | side button with volume up, saved to Photos; also from a computer over USB |
| USB | seen as an iPhone by a computer: pairing, the usual iPhone tools, the system log, screenshots |
| Crash reports | a crashed process leaves Apple's usual `.ips` report; `idevicecrashreport -k DIR` on a computer collects them |

### Metal, in detail

`MetalTest`, a test application that comes with the image, draws 25 pages, one feature each. All 25 are right:

clears, vertex buffers, indexed draws (16 and 32 bit), textures (nearest, linear, mipmaps, generated mipmaps), alpha
blending, depth, stencil, instancing, 4x multisampling, float render targets, several render targets in one pass,
compute kernels, ASTC textures (sRGB included), array / cube / 3D textures, the common pixel formats, points with a
size, lines, strips, viewports and scissors, discard and derivatives, blit copies and read-back, 400 draws a frame,
layered rendering (one draw into several layers of an array texture, colour or depth), passes of depth alone
(shadow maps: a pipeline with no fragment function), UIKit views animating over a Metal layer.

## Works in part

| | |
|---|---|
| Metal: layered rendering | up to 8 layers, colour and depth; not yet with stencil, multisampling, several colour targets, or a pass that loads what was there |
| Battery life | poor: the phone never sleeps (no suspend yet); about 0.47 A with the screen off. Idle cores are powered down and the unused power domains turned off; Wi-Fi's PCIe link (no low-power state yet) is 0.06 A of it, where the rest goes is not found yet |
| Passcode | not tied to the phone's hardware: its keys are in a file on the data volume, not in the secure chip |
| Microphone | reaches the kernel; iOS does not record from it yet |
| Camera | the back camera (the main sensor, Sony IMX787) and the front camera (Sony IMX712) are iOS's cameras: the Camera application shows the live picture, takes photographs and flips between the two (about half a second). Not yet: the ultra-wide, focus, colour and white balance tuning (pictures are tinted), video recording |
| Modem | the Exynos 5300 is started by our kernel (`modem=9`, off by default): PCIe link at Gen3 on two lanes, its firmware read from the phone's own `modem_a` partition and loaded, the start handshake, its file service answered, the link put to sleep and woken as the modem asks. It answers requests in its command protocol (radio power, baseband version, SIM state); with the radio on (`modem_radio=1`) and no SIM it hears a network and reports the signal. Not done: registration with a SIM, data, calls and messages; nothing of it shows in iOS yet (the kernel keeps `sysctl kern.hakios.cellular_*` for it) |

## Not yet

- Mobile network (calls, SMS, data), GPS, NFC
- Motion and light sensors: rotation, auto-brightness, proximity, compass
- Sleep (suspend to RAM)
- Fingerprint reader
- Apple ID sign-in and the App Store; installing applications; Weather has no data (Apple's weather service wants a signed-in device)
- Setup Assistant on first start, factory reset from Settings
- Photos to Windows over USB

## Known rough edges

- The first start after a new image compiles shaders it has not seen before: a few seconds of missing pieces on
  screen, once.
- Touch scrolling feels faster than on Android.
- Opening Spotlight drops to 30-40 frames a second for half a second.

## How it is put together

- **Start:** slot b holds one boot image: U-Boot, our loader, the XNU kernel, a small RAM disk with our programs,
  the drivers' firmware and a seed of compiled shaders. The `darwin` partition holds Apple's root volume as it comes
  from the IPSW, the cryptexes, and a data volume for `/private/var`.
- **Apple's files are not changed on disk.** What has to differ is put in place at start (`userland/src/init.c`:
  bind mounts and synthetic directories) and by small libraries inserted into a few daemons (`userland/src/inject*.c`).
- **Drivers** for this phone are in `drivers/` (display, touch, GPU, storage, USB, Wi-Fi, audio, power, thermal ...).
- **Bluetooth:** the kernel switches the BCM4383 on and loads its firmware (`drivers/bluetooth/` here), and gives the
  controller to iOS's own `bluetoothd` through the chip-independent half in the main repository (`drivers/bluetooth/`:
  the kernel control and Skywalk pipe `bluetoothd` opens, from upstream's MediaTek work); BlueTool's service is
  answered by a small inserted library (`userland/src/inject_bt.c`).
- **Graphics:** `hakios-metal/` in the main repository is a Metal driver of our own. Its Mali back end writes the
  GPU's command streams directly (Mesa's panvk is the reference) and compiles shaders with Mesa's Panfrost compiler
  from the Metal libraries' bitcode; `userland/src/HakiosMetalCA.m` is the Metal plug-in iOS loads.
- **Testing** is over the USB cable (`tools/`: logs, one-frame screenshots, restart, shader cache) with test boots from
  RAM that flash nothing (`tools/bootrun.sh`), driven by job files of taps and test programs (`userland/rc.jobs.*`).

### For a port to another phone

Carries over as it is: the loader and U-Boot path, the way of running Apple's untouched root volume, the inserted
libraries, the USB side that answers as an iPhone, the software passcode store, the Metal plug-in above the driver's C
interface, the shader front end, MetalTest, the tools and job files.

Things that cost days here and are the same on any phone:

- Nothing long in a kernel timer callback (thread call) that does not block: it holds up every other timer of the
  system. A Wi-Fi firmware load done there froze the display for a second.
- A missing Apple service must answer "not there", not stay silent or say yes with nothing: SpringBoard retrying a
  telephony daemon that never answers reset the idle timer every second (no auto-lock); a sandbox call answered
  "success" with an empty buffer made Safari's history service abort at every start.
- "Setup is done" is asked by many processes, not only SpringBoard (`purplebuddy.sentinel`, see
  `userland/src/inject_sb.c`): without it no button screenshots and no deferred background work.
- The bootloader leaves every power domain on; turn off what no driver uses (`drivers/power/hakios.akita_power.c`).
- A test boot from RAM that resets early falls into the installed image. Keep that in mind for what is flashed.
- When a chip stays silent and the vendor's driver source does not say why, trace the vendor's kernel on the phone:
  Android's `events/rwmmio` logs every register it writes. That showed in one run that Bluetooth's power pin goes
  through a latch (a second pin is pulsed after each change), which no public source for this phone says.
- Tools that found most faults: the kernel log beside a process's own times, `tools/dsc-backtrace.py` and
  `tools/dsc-disasm.py` on the shared cache (names for an abort's frames, then the code around them).

To write anew for another chip: everything in `drivers/`, and for a different GPU family the part of `hakios-metal`
below its C interface. The Mali back end here and the Adreno one beside it show what that takes.

# Pixel 8a (akita): status

HakiOS (iOS 16.7.16 userland on XNU) on the Google Pixel 8a (Tensor G3, Mali-G715). Last update: 2026-11-14.

The phone starts iOS from its own storage (slot b) in about 45 seconds; Android stays on slot a. Everything below is
what was seen on the one phone this port is developed on.

## Works

| | |
|---|---|
| Boot | from the phone's storage, straight to the lock or home screen; iOS's data is kept between starts |
| Safety net | a start that does not reach the home screen in 4 minutes restarts into the bootloader (fastboot) |
| Screen | 1080 x 2400 at 120 Hz, drawn by the GPU; brightness slider; screen off and on with the side button |
| Touch and buttons | touch, power, volume |
| GPU | Mali-G715 with a Metal driver of our own: the system's interface, the applications, Safari and WebGL draw with it |
| Wi-Fi | joins WPA2 networks, DHCP, HTTPS; each phone has its own address |
| Sound | both speakers |
| Vibration | yes |
| Flashlight | the LED works (Control Center's button not confirmed yet) |
| Battery | charge level and charging shown |
| Heat and speed | all 9 cores, speed scaled with load and temperature, cores idle when there is nothing to do |
| Clock | keeps the time |
| Shut down / restart | yes |
| Passcode | can be set, changed and removed in Settings; the phone locks and asks for it (kept in software, see below) |
| Stock applications | all installed, the removable ones (Mail, Maps, Notes, ...) included; those of the first page and the dock were opened and draw, the rest not yet gone through |
| USB | seen as an iPhone by a computer: pairing, the usual iPhone tools, the system log, screenshots |

### Metal, in detail

`MetalTest`, a test application that comes with the image, draws 23 pages, one feature each. All 23 are right:

clears, vertex buffers, indexed draws (16 and 32 bit), textures (nearest, linear, mipmaps, generated mipmaps), alpha
blending, depth, stencil, instancing, 4x multisampling, float render targets, several render targets in one pass,
compute kernels, ASTC textures (sRGB included), array / cube / 3D textures, the common pixel formats, points with a
size, lines, strips, viewports and scissors, discard and derivatives, blit copies and read-back, 400 draws a frame,
layered rendering (one draw into several layers of an array texture), UIKit views animating over a Metal layer.

## Works in part

| | |
|---|---|
| Metal: layered rendering | up to 8 layers of one colour attachment; not yet with a depth attachment (shadow cube maps in one pass), multisampling, or a pass that loads what was there |
| Metal: frame rate | an application's own Metal layer shows 75-105 frames a second, not the panel's 120 (its drawables come back late; the drawing itself takes a fraction of a millisecond) |
| WebGL | works; heavy pages are slow (a 600-draw aquarium demo: about 30 frames a second) |
| Battery life | poor: the phone never sleeps (no suspend yet); about 0.5 A with the screen off |
| Passcode | not tied to the phone's hardware: its keys are in a file on the data volume, not in the secure chip |
| Microphone | reaches the kernel; iOS does not record from it yet |
| Camera | both sensors answer; no picture yet |
| Screenshots | from a computer over USB; the button combination does nothing yet |

## Not yet

- Bluetooth
- Mobile network (calls, SMS, data), GPS, NFC
- Motion and light sensors: rotation, auto-brightness, proximity, compass
- Sleep (suspend to RAM)
- Fingerprint reader
- Apple ID sign-in and the App Store; installing applications
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
- **Graphics:** `hakios-metal/` in the main repository is a Metal driver of our own. Its Mali back end writes the
  GPU's command streams directly (Mesa's panvk is the reference) and compiles shaders with Mesa's Panfrost compiler
  from the Metal libraries' bitcode; `userland/src/HakiosMetalCA.m` is the Metal plug-in iOS loads.
- **Testing** is over the USB cable (`tools/`: logs, one-frame screenshots, restart, shader cache) with test boots from
  RAM that flash nothing (`tools/bootrun.sh`), driven by job files of taps and test programs (`userland/rc.jobs.*`).

### For a port to another phone

Carries over as it is: the loader and U-Boot path, the way of running Apple's untouched root volume, the inserted
libraries, the USB side that answers as an iPhone, the software passcode store, the Metal plug-in above the driver's C
interface, the shader front end, MetalTest, the tools and job files.

To write anew for another chip: everything in `drivers/`, and for a different GPU family the part of `hakios-metal`
below its C interface. The Mali back end here and the Adreno one beside it show what that takes.

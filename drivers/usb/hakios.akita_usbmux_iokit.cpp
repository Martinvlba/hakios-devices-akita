/*
 * The USB mux's presence in IOKit, as lockdownd looks for it. On an iPhone IOUSBDeviceFamily makes an
 * IOUSBDeviceInterface for each function on the port, and lockdownd learns that a computer is there from them: its
 * detect_connected_usb_host (and a matching notification) takes every IOUSBDeviceInterface, and one whose
 * USBDeviceFunction is "AppleUSBMux" (or "PTP") with IsActive true means a host on the cable -- usb_host_connected,
 * in mux mode for AppleUSBMux (read from its code: IOServiceGetMatchingServices, IORegistryEntryCreateCFProperties,
 * CFEqual against those constants and kCFBooleanTrue). Without one it refused to ask the user to trust a computer:
 * "ask_user_to_trust: USB or proxy host no longer connected", and the pairing failed with InvalidHostID. An
 * AppleUSBDeviceMux (the mux kext's own class, which lockdownd also names) is published with it.
 *
 * The mux itself is hakios.akita_usb.c and hakios.akita_usbmux.c; this is only its presence in IOKit: published under
 * the platform expert when the host configures the mux interface (SET_CONFIGURATION 1), terminated when it goes
 * (deconfigured, reset, disconnected). Nothing opens it.
 */

#include <IOKit/IOLib.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/IOService.h>
#include <libkern/c++/OSString.h>

extern "C" void akita_usbmux_iokit(int connected);

class AppleUSBDeviceMux : public IOService
{
	OSDeclareDefaultStructors(AppleUSBDeviceMux);
};

OSDefineMetaClassAndStructors(AppleUSBDeviceMux, IOService);

class IOUSBDeviceInterface : public IOService
{
	OSDeclareDefaultStructors(IOUSBDeviceInterface);
};

OSDefineMetaClassAndStructors(IOUSBDeviceInterface, IOService);

static AppleUSBDeviceMux * mux;
static IOUSBDeviceInterface * interface;

/* a service of ours under the platform expert, registered; NULL if it could not be */
template <class T>
static T *
publish(IOService * platform, bool active)
{
	T * made = OSTypeAlloc(T);

	if (made == NULL || !made->init()) {
		OSSafeReleaseNULL(made);
		return NULL;
	}
	made->setProperty("HakiosUSBMux", "devices/akita/drivers/usb");
	made->setProperty("USBDeviceFunction", "AppleUSBMux");
	made->setProperty("IsActive", active);
	if (!made->attach(platform)) {
		made->release();
		return NULL;
	}
	if (!made->start(platform)) {
		made->detach(platform);
		made->release();
		return NULL;
	}
	made->registerService();
	return made;
}

/* from the USB driver's poll thread, when the host configures the mux interface (1) or is gone (0) */
void
akita_usbmux_iokit(int connected)
{
	IOService * platform = IOService::getPlatform();

	if (connected && interface == NULL && platform != NULL) {
		mux = publish<AppleUSBDeviceMux>(platform, true);
		interface = publish<IOUSBDeviceInterface>(platform, true);
		IOLog("akita usbmux: IOUSBDeviceInterface (AppleUSBMux, active)%s published: a host is on the cable\n",
		    mux ? " and AppleUSBDeviceMux" : "");
	} else if (!connected && interface != NULL) {
		IOUSBDeviceInterface * gone = interface;

		interface = NULL;
		gone->terminate();
		gone->release();
		if (mux != NULL) {
			AppleUSBDeviceMux * m = mux;

			mux = NULL;
			m->terminate();
			m->release();
		}
		IOLog("akita usbmux: IOUSBDeviceInterface terminated: the host is gone\n");
	}
}

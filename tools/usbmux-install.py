#!/usr/bin/env python3
"""Install an application on the phone over the USB cable, the way iTunes/Xcode/ideviceinstaller do it: lockdownd
(port 62078, the PC's pairing) starts the services, AFC takes the package into /var/mobile/Media/PublicStaging, and
com.apple.mobile.installation_proxy has installd install it.

usage: usbmux-install.py services                 which of the services lockdownd will start
       usbmux-install.py list [user|system|any]   the applications installd knows (bundle id, version, path)
       usbmux-install.py install APP.ipa | X.app  upload and install (a .app directory is zipped into an .ipa first)
       usbmux-install.py uninstall BUNDLE.ID
       usbmux-install.py lookup BUNDLE.ID         everything installd says about one application

(needs usbmuxd running and this PC paired: `idevicepair pair`. Nothing of the phone's is written anywhere: the pair
record is read from usbmuxd each time.)
"""
import os
import plistlib
import socket
import ssl
import struct
import sys
import tempfile
import zipfile

MUX = '/var/run/usbmuxd'


def mux_request(message):
    s = socket.socket(socket.AF_UNIX)
    s.connect(MUX)
    body = plistlib.dumps(message)
    s.sendall(struct.pack('<IIII', 16 + len(body), 1, 8, 1) + body)
    head = b''
    while len(head) < 16:
        part = s.recv(16 - len(head))
        if not part:
            sys.exit('usbmux-install: usbmuxd closed the connection')
        head += part
    n = struct.unpack('<I', head[:4])[0]
    data = b''
    while len(data) < n - 16:
        data += s.recv(n - 16 - len(data))
    return s, plistlib.loads(data)


def recv_all(s, n):
    data = b''
    while len(data) < n:
        part = s.recv(n - len(data))
        if not part:
            raise EOFError('the phone closed the connection after %d of %d bytes' % (len(data), n))
        data += part
    return data


def send_plist(s, message, fmt=plistlib.FMT_XML):
    body = plistlib.dumps(message, fmt=fmt)
    s.sendall(struct.pack('>I', len(body)) + body)


def recv_plist(s):
    return plistlib.loads(recv_all(s, struct.unpack('>I', recv_all(s, 4))[0]))


class Phone:
    def __init__(self):
        s, listing = mux_request({'MessageType': 'ListDevices'})
        s.close()
        devices = [d for d in listing.get('DeviceList', []) if d.get('Properties', {}).get('ConnectionType') == 'USB']
        if not devices:
            sys.exit('usbmux-install: no device (is usbmuxd running, the phone attached?)')
        self.device = devices[0]['DeviceID']
        udid = devices[0]['Properties']['SerialNumber']
        s, answer = mux_request({'MessageType': 'ReadPairRecord', 'PairRecordID': udid})
        s.close()
        if 'PairRecordData' not in answer:
            sys.exit('usbmux-install: this PC is not paired with the phone (idevicepair pair)')
        self.pair = plistlib.loads(answer['PairRecordData'])
        self.dir = tempfile.TemporaryDirectory()
        self.cert = os.path.join(self.dir.name, 'host.pem')
        fd = os.open(self.cert, os.O_WRONLY | os.O_CREAT, 0o600)
        os.write(fd, self.pair['HostCertificate'] + b'\n' + self.pair['HostPrivateKey'])
        os.close(fd)

    def connect(self, port):
        s, answer = mux_request({'MessageType': 'Connect', 'DeviceID': self.device, 'PortNumber': socket.htons(port)})
        if answer.get('Number', 1) != 0:
            s.close()
            raise ConnectionError('usbmuxd: no connection to the phone\'s port %d (result %s)' % (port, answer.get('Number')))
        return s

    def tls(self, s):
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
        try:
            context.set_ciphers('ALL:@SECLEVEL=0')
        except ssl.SSLError:
            pass
        context.load_cert_chain(self.cert)
        return context.wrap_socket(s)

    def lockdown(self):
        s = self.connect(62078)
        send_plist(s, {'Label': 'usbmux-install', 'Request': 'QueryType'})
        kind = recv_plist(s)
        if kind.get('Type') != 'com.apple.mobile.lockdown':
            raise ConnectionError('port 62078 is not lockdownd: %r' % kind)
        send_plist(s, {'Label': 'usbmux-install', 'Request': 'StartSession', 'HostID': self.pair['HostID'],
                       'SystemBUID': self.pair['SystemBUID']})
        session = recv_plist(s)
        if 'Error' in session:
            raise ConnectionError('lockdownd: StartSession: %s' % session['Error'])
        if session.get('EnableSessionSSL'):
            s = self.tls(s)
        return s

    def service(self, name):
        """a connection to a lockdown service, or the error lockdownd gave"""
        s = self.lockdown()
        try:
            send_plist(s, {'Label': 'usbmux-install', 'Request': 'StartService', 'Service': name})
            answer = recv_plist(s)
        finally:
            s.close()
        if 'Error' in answer or 'Port' not in answer:
            raise ConnectionError('lockdownd will not start %s: %s' % (name, answer.get('Error', answer)))
        c = self.connect(answer['Port'])
        if answer.get('EnableServiceSSL'):
            c = self.tls(c)
        return c


# AFC: 40-byte header ("CFA6LPAA", whole length, this-part length, packet number, operation), then the operation's
# header data and its payload
AFC_STATUS, AFC_DATA, AFC_MKDIR, AFC_OPEN, AFC_OPENED, AFC_WRITE, AFC_CLOSE, AFC_REMOVE = 1, 2, 9, 13, 14, 16, 20, 8


class Afc:
    def __init__(self, s):
        self.s = s
        self.number = 0

    def call(self, operation, header=b'', payload=b''):
        self.number += 1
        self.s.sendall(b'CFA6LPAA' + struct.pack('<QQQQ', 40 + len(header) + len(payload), 40 + len(header), self.number,
                                                 operation) + header + payload)
        head = recv_all(self.s, 40)
        whole, this, _, answer = struct.unpack('<QQQQ', head[8:])
        data = recv_all(self.s, whole - 40)
        if answer == AFC_STATUS:
            status = struct.unpack('<Q', data[:8])[0]
            if status != 0:
                raise OSError('AFC operation %d: status %d' % (operation, status))
        return answer, data

    def mkdir(self, path):
        try:
            self.call(AFC_MKDIR, path.encode() + b'\0')
        except OSError:
            pass                    # (there already)

    def put(self, local, remote):
        _, data = self.call(AFC_OPEN, struct.pack('<Q', 3) + remote.encode() + b'\0')    # 3: write, create, truncate
        handle = data[:8]
        sent = 0
        with open(local, 'rb') as f:
            while True:
                part = f.read(1 << 20)
                if not part:
                    break
                self.call(AFC_WRITE, handle, part)
                sent += len(part)
        self.call(AFC_CLOSE, handle)
        return sent


def proxy_command(phone, command, progress=None):
    """one command to installation_proxy; its answers until Complete or an Error"""
    s = phone.service('com.apple.mobile.installation_proxy')
    answers = []
    try:
        send_plist(s, command)
        while True:
            answer = recv_plist(s)
            answers.append(answer)
            if 'Error' in answer:
                raise RuntimeError('installd: %s: %s' % (answer['Error'], answer.get('ErrorDescription', '')))
            if progress is not None and 'PercentComplete' in answer:
                progress('%3d%% %s' % (answer['PercentComplete'], answer.get('Status', '')))
            if answer.get('Status') == 'Complete':
                break
    finally:
        s.close()
    return answers


def make_ipa(app, out):
    name = os.path.basename(app.rstrip('/'))
    with zipfile.ZipFile(out, 'w', zipfile.ZIP_DEFLATED) as z:
        for root, _, files in os.walk(app):
            for f in files:
                path = os.path.join(root, f)
                info = zipfile.ZipInfo.from_file(path, os.path.join('Payload', name, os.path.relpath(path, app)))
                info.compress_type = zipfile.ZIP_DEFLATED
                with open(path, 'rb') as data:
                    z.writestr(info, data.read())       # (ZipInfo.from_file keeps the mode: the executable's x bit)


def main():
    what = sys.argv[1] if sys.argv[1:] else ''
    phone = Phone()
    if what == 'services':
        for name in ('com.apple.afc', 'com.apple.mobile.installation_proxy', 'com.apple.mobile.house_arrest',
                     'com.apple.streaming_zip_conduit', 'com.apple.misagent', 'com.apple.springboardservices',
                     'com.apple.mobile.notification_proxy', 'com.apple.syslog_relay', 'com.apple.crashreportcopymobile',
                     'com.apple.mobile.diagnostics_relay', 'com.apple.mobile.screenshotr', 'com.apple.debugserver'):
            try:
                phone.service(name).close()
                print('%-44s starts' % name)
            except Exception as e:
                print('%-44s %s' % (name, e))
    elif what == 'list':
        kind = {'user': 'User', 'system': 'System', 'any': 'Any'}.get(sys.argv[2] if sys.argv[2:] else 'user', 'User')
        answers = proxy_command(phone, {'Command': 'Browse', 'ClientOptions': {
            'ApplicationType': kind, 'ReturnAttributes': ['CFBundleIdentifier', 'CFBundleShortVersionString', 'Path']}})
        n = 0
        for a in answers:
            for app in a.get('CurrentList', []):
                n += 1
                print('%-44s %-10s %s' % (app.get('CFBundleIdentifier'), app.get('CFBundleShortVersionString', ''),
                                          app.get('Path', '')))
        print('usbmux-install: %d application(s) of type %s' % (n, kind))
    elif what == 'lookup' and sys.argv[2:]:
        s = phone.service('com.apple.mobile.installation_proxy')
        send_plist(s, {'Command': 'Lookup', 'ClientOptions': {'BundleIDs': [sys.argv[2]]}})
        answer = recv_plist(s)
        s.close()
        for key, value in sorted(answer.get('LookupResult', {}).get(sys.argv[2], {}).items()):
            print('%-36s %s' % (key, str(value)[:200]))
        if not answer.get('LookupResult'):
            print('usbmux-install: installd knows no %s (%s)' % (sys.argv[2], answer))
    elif what == 'install' and sys.argv[2:]:
        package = sys.argv[2]
        temporary = None
        if os.path.isdir(package):
            temporary = tempfile.NamedTemporaryFile(suffix='.ipa', delete=False)
            temporary.close()
            make_ipa(package, temporary.name)
            package = temporary.name
        remote = 'PublicStaging/' + os.path.basename(sys.argv[2].rstrip('/')).replace('.app', '') + '.ipa'
        if not remote.endswith('.ipa'):
            remote += '.ipa'
        afc = Afc(phone.service('com.apple.afc'))
        afc.mkdir('PublicStaging')
        sent = afc.put(package, remote)
        afc.s.close()
        if temporary is not None:
            os.unlink(temporary.name)
        print('usbmux-install: %d bytes in /var/mobile/Media/%s' % (sent, remote))
        proxy_command(phone, {'Command': 'Install', 'PackagePath': remote, 'ClientOptions': {}},
                      progress=lambda line: print('usbmux-install:', line))
        print('usbmux-install: installed')
    elif what == 'uninstall' and sys.argv[2:]:
        proxy_command(phone, {'Command': 'Uninstall', 'ApplicationIdentifier': sys.argv[2]},
                      progress=lambda line: print('usbmux-install:', line))
        print('usbmux-install: uninstalled')
    else:
        sys.exit(__doc__)


if __name__ == '__main__':
    try:
        main()
    except (ConnectionError, RuntimeError, OSError, EOFError) as e:
        sys.exit('usbmux-install: %s' % e)

# python -m pip install spur
# python -m pip install pysftp
# c:\Users\halil\.atom\packages\platformio-ide\penv\Scripts>pip.exe install spur
# c:\Users\halil\.atom\packages\platformio-ide\penv\Scripts>pip.exe install pysftp

import pysftp
import spur
import sys
from CONFIG import * # host, username, password and port is in CONFIG.py

which = sys.argv[1]

print "UPLOADING TO SERVER: ", which

path_to_firmware = '.pioenvs/nucleo_f042k6_' + which + '/firmware.bin'
target_path_to_firmware = '/home/pi/firmware.bin'

cnopts = pysftp.CnOpts()
cnopts.hostkeys = None    # disable host key checking.
cinfo = {'host':host, 'username':username, 'password':password, 'port':port, 'cnopts':cnopts}
with pysftp.Connection(**cinfo) as sftp:
  sftp.put(path_to_firmware, target_path_to_firmware)

print "UPLOAD COMPLETED!!"

shell = spur.SshShell(hostname=host, username=username, password=password, port=port, missing_host_key=spur.ssh.MissingHostKey.accept)
result = shell.run(["st-flash", "write", target_path_to_firmware, "0x08000000"])
print result.output

print "FLASH COMPLETED!!"

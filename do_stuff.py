Import('env')
from base64 import b64decode

upload_to_server = b64decode(ARGUMENTS.get("UPLOAD_TO_SERVER")) == '1'
incr_version = b64decode(ARGUMENTS.get("INCR_VERSION")) == '1'

if incr_version:
  f = open("last_version")
  s = f.read()
  ss = s.split(',')
  v = [int(ss[0]), int(ss[1]), int(ss[2])]
  f.close()
  v[2] += 1
  v = map(str, v)
  print v[0] + "," + v[1] + "," + v[2]
  f = open("last_version", "w")
  f.write(v[0] + "," + v[1] + "," + v[2])
  f.close()
  f = open("lib/mylib/version.hpp", "w")
  f.write('''#ifndef VERSION_HPP
#define VERSION_HPP

#define MAIN_VERSION %s
#define SUB_VERSION %s
#define SUB_SUB_VERSION %s

#endif''' % (v[0], v[1], v[2]))

if upload_to_server:
  env.Replace(UPLOADCMD='python upload_to_server.py ' + b64decode(ARGUMENTS.get("UPLOAD_WHICH_FILE")))

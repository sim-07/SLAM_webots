import os
from SCons.Script import DefaultEnvironment

env = DefaultEnvironment()

webots_home = os.environ.get("WEBOTS_HOME", "/usr/local/webots")

webots_lib = os.path.join(webots_home, "lib", "controller")

env.Append(LIBPATH=[webots_lib])

env.Append(LIBS=["Controller", "CppController"])

env.Append(LINKFLAGS=["-Wl,-rpath," + webots_lib])
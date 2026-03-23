Import("env")

import subprocess
import hex2sysex

def after_build(source, target, env):
    # python tools/hex2sysex.py .pio/build/app/firmware.hex > update.syx
    with open("app-update.syx", "wb") as outfile:
        hex2sysex.process(".pio/build/app/firmware.hex", outfile)
        # subprocess.run(["python", "tools/hex2sysex.py", ".pio/build/app/firmware.hex"], stdout=outfile)

env.AddPostAction("buildprog", after_build)

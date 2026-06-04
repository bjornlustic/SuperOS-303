Import("env")

# The atmelavr builder forwards build_flags only to the COMPILE step, not the
# link. Put the freestanding flags on LINKFLAGS so crt0 / the startup vectors
# are NOT linked: the SPM service is pure callable code in the boot section with
# no main(), no reset vector. LTO is disabled so the entry trampoline's inline
# `jmp flash_service_impl` resolves to a plain symbol relocation.
env.Append(LINKFLAGS=["-nostartfiles", "-nostdlib"])
env.Append(CCFLAGS=["-fno-lto"])
env.Append(LINKFLAGS=["-fno-lto"])

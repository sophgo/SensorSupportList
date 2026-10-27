import os
import subprocess
import sys

# Only RT-Thread builds define the `building` module; other SDKs
# (e.g. AliOS/yoctools) also traverse this repo and would die on the
# import. In that case fall through without registering anything
# (SCons allows SConscripts to end without an explicit Return).
try:
    from building import *
    _IS_RTTHREAD_BUILD = True
except ImportError:
    _IS_RTTHREAD_BUILD = False

if _IS_RTTHREAD_BUILD:

    cwd = GetCurrentDir()
    src     = []
    CPPDEFINES = []
    CPPPATH = []

    # ---- Keep Kconfig.sensors in sync with the repo's driver dirs ----
    # (generated build artifact; see gen_sensor_config.py)
    _gen = os.path.join(cwd, "gen_sensor_config.py")
    try:
        subprocess.check_call([sys.executable, _gen])
    except Exception as e:
        print("SensorSupportList: gen_sensor_config failed: %s" % e)

    # ---- Read the selected sensor set from rtconfig.h (bare macros) ----
    SENSORS = []
    for entry in sorted(os.listdir(cwd)):
        path = os.path.join(cwd, entry)
        if not os.path.isdir(path):
            continue
        if entry in ("CViTEK_Sensor_Support_List", "sensor_cfg", "sensor_i2c",
                     "sensor_list", "rtt", ".git"):
            continue
        if not any(f.endswith(".c") for f in os.listdir(path)):
            continue
        SENSORS.append(entry)

    selected = [d for d in SENSORS if GetDepend("SENSOR_" + d.upper())]

    # Re-inject the prefixed CONFIG_SENSOR_* form: RT-Thread's mk_rtconfig
    # strips the CONFIG_ prefix, but the repo lookup tables are guarded by
    # #if defined(CONFIG_SENSOR_<DIR>).
    for d in selected:
        CPPDEFINES += ["CONFIG_SENSOR_%s=1" % d.upper()]
    CPPDEFINES += ["SENSOR_SUPPORTLIST=1"]
    CPPDEFINES += ["__RTTHREAD__=1"]   # sensor_i2c RT backend

    # ---- Build set: every selected driver + the shared core ----
    for d in selected:
        src += Glob(os.path.join(d, "*.c"))
    src += Glob(os.path.join("sensor_cfg", "*.c"))
    src += Glob(os.path.join("sensor_i2c", "sensor_i2c.c"))

    # Consumer-facing include paths (board param files, media layer, samples):
    #   sensor_cfg.h / cvi_sns_ctrl.h types come from the repo core; the
    #   RT shim header is included as <sensor_cfg.h> would be on Linux.
    CPPPATH += [
        cwd,
        os.path.join(cwd, "sensor_cfg"),
        os.path.join(cwd, "sensor_i2c"),
        os.path.join(cwd, "..", "cvi_mpi", "include"),
        os.path.join(cwd, "..", "..", "..", "bsp", "cvitek", "cvi_common", "cvi_hal"),
    ]

    # Boards that have not opted into this mechanism (no SENSOR_* in
    # rtconfig.h) must see no build impact at all: no core compilation, no
    # macro injection. Stay silent and empty so unrelated SDK builds
    # (other chips/solutions that happen to traverse cvi_comps) are
    # unaffected.
    if not selected:
        # No SENSOR_* selected: compile nothing, inject nothing —
        # builds that have not opted in see no impact at all.
        Return('')

    sns = DefineGroup('SensorSupportList', src, depend = [''],
                      CPPDEFINES = CPPDEFINES, CPPPATH = CPPPATH)

    Return('sns')

else:
    # AliOS (yoctools) traverses this repo as a component too (see
    # package.yaml). Its build_component only auto-generates a SConscript
    # stub (Import('defconfig'); defconfig.library_yaml()) when none
    # exists; ours supersedes it, so replicate that stub here to keep
    # building libSensorSupportList.a for those SDKs.
    Import('defconfig')
    defconfig.library_yaml()

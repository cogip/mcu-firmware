#!/usr/bin/env python3
import ctypes

import sysv_ipc


class ShmData(ctypes.Structure):
    _fields_ = [
        ('tof', ctypes.c_ushort * 6),
        ('lidar', ctypes.c_ushort * 360)
    ]


shm_ptr = sysv_ipc.SharedMemory(
    key=None, mode=0o666, flags=sysv_ipc.IPC_CREX,
    size=ctypes.sizeof(ShmData)
)
shm_data = ShmData.from_buffer(shm_ptr)

for i in range(360):
    shm_data.lidar[i] = 65535

input(
    f"""
Shared memory key = {shm_ptr.key}

Start the native application with gdb.
In the RIOT shell:
    - use the key with '_set_shmem_key',
    - start the planner (pln_menu -> G)

Hit enter to add an obstacle.
""")

shm_data.lidar[0] = 700

input('Press ENTER to exit...')

shm_ptr.detach()
shm_ptr.remove()

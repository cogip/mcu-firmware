import ctypes

import sysv_ipc


NB_DATA = 8


class ShmData(ctypes.Structure):
    _fields_ = [
        ('even', ctypes.c_uint8 * NB_DATA),
        ('odd', ctypes.c_uint16 * NB_DATA)
    ]


shm_ptr = sysv_ipc.SharedMemory(
    key=None, mode=0o666, flags=sysv_ipc.IPC_CREX,
    size=ctypes.sizeof(ShmData)
)
shm_data = ShmData.from_buffer(shm_ptr)

for i in range(NB_DATA):
    shm_data.even[i] = 2 * i
    shm_data.odd[i] = 2 * i + 1

print("Shared memory initialized with:")
print(f"data.even = {list(shm_data.even)}")
print(f"data.odd  = {list(shm_data.odd)}")
print(f"Shm key = {shm_ptr.key}")

input('Press ENTER to exit...')

shm_ptr.detach()
shm_ptr.remove()

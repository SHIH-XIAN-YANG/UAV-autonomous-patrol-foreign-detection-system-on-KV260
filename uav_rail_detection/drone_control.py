import numpy as np
from dronekit import connect, VehicleMode
import time

from multiprocessing import shared_memory

shm = shared_memory.SharedMemory(name='control command')

control_command = ()


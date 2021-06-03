# /usr/bin/env python3

from pyModbusTCP.client import ModbusClient
import time
import math

#ModBus addresses
pressure_addr = 256
vacuum_addr = 257
set_pressure_addr = 775
read_pressure_addr = 512
read_vacuum_addr = 513

class RochuGripper:
    def __init__(self,ip,port):
        #initialise modbustcp object
        self.c = ModbusClient()
        self.c.host(ip)
        self.c.port(port)
    
    def trigger_pressure(self):
        toggle = True
        return self.c.write_single_coil(pressure_addr, toggle)
        
    def cancel_pressure(self):
        toggle = False
        return self.c.write_single_coil(pressure_addr, toggle)
    
    def trigger_vacuum(self):
        toggle = True
        return self.c.write_single_coil(vacuum_addr, toggle)

    def cancel_vacuum(self):
        toggle = False
        return self.c.write_single_coil(vacuum_addr, toggle)
    
    def set_pressure_value(self,percentage,max_effort=120,min_effort=0):
        #0.05MPa/V
        pressure_range = max_effort*(10**3) - min_effort*(10**3)
        lower_bound_votlage = min_effort*(10**3)/((10**3)*0.05)
        voltage_range = pressure_range/((10**3)*0.05)
        #voltage send in mV
        voltage = round(lower_bound_votlage+(voltage_range*percentage/100))
        last_pressure = round(min_effort + (pressure_range*percentage/100))
        return self.c.write_single_register(set_pressure_addr,voltage) , voltage 

    def read_pressure_feedback(self):
        #True if pressure more than P_1 value set on ACU
        pressure = self.c.read_discrete_inputs(read_pressure_addr,1)
        return pressure[0]
    
    def read_vacuum_feedback(self):
        #True if pressure more than n_2 value set on ACU
        vacuum = self.c.read_discrete_inputs(read_vacuum_addr,1)
        return vacuum[0]
    
    def get_gripper_state(self):
        #MODE_GRAB
        if self.read_pressure_feedback() and not self.read_vacuum_feedback():
            return 0
        #MODE_RELEASE
        elif self.read_vacuum_feedback() and not self.read_pressure_feedback():
            return 2
        #MODE_IDLE
        elif not self.read_pressure_feedback() and not self.read_vacuum_feedback():
            return 1
        #MODE NOT DETERMINED
        else :
            return 3

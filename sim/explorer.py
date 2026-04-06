# explorer.py - Introspect Renode's Python API
from Antmicro.Renode.Logging import Logger, LogLevel

def log(msg):
    # Try multiple ways to log
    print(msg)
    try:
        if 'machine' in globals():
            machine.Log(LogLevel.Info, "Explorer: " + msg)
    except:
        pass

log("Explorer starting...")
log("Globals: " + str(sorted(globals().keys())))

if 'self' in globals():
    log("self type: " + str(self.GetType().FullName))
    log("self members: " + str(sorted(dir(self))))
    if hasattr(self, 'Machine'):
        m = self.Machine
        log("Found machine via self.Machine")
    else:
        m = None
else:
    m = None

if m is None and 'emulationManager' in globals():
    try:
        emulation = emulationManager.Instance.CurrentEmulation
        if emulation.MachinesCount > 0:
            m = emulation.Machines[0]
            log("Found machine via emulationManager")
    except:
        pass

if m:
    log("Machine type: " + str(m.GetType().FullName))
    log("Machine members: " + str(sorted(dir(m))))
    
    # Try looking for SystemBus
    if hasattr(m, 'SystemBus'):
        sb = m.SystemBus
        log("SystemBus type: " + str(sb.GetType().FullName))
        log("SystemBus members: " + str(sorted(dir(sb))))
        
        # Try finding timers in Children
        try:
            log("SystemBus Children: " + str([c.Name for c in sb.Children]))
        except:
            pass

    # Try getting a timer via alternate methods
    try:
        log("All names: " + str(list(m.GetAllNames())))
    except:
        pass
        
    try:
        peripherals = list(sb.GetRegisteredPeripherals())
        log("Registered peripherals: " + str([p.Name for p in peripherals]))
    except:
        pass
    
    if t1:
        log("TIM1 type: " + str(t1.GetType().FullName))
        log("TIM1 members: " + str(sorted(dir(t1))))

log("Explorer finished.")

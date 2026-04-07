# Renode script to simulate STM32 Timer TRGO/Slave sync for Renode 1.16.1
from Antmicro.Renode.Logging import Logger, LogLevel
from Antmicro.Renode.Time import TimeInterval
from Antmicro.Renode.Core import EmulationManager
import __builtin__

class TimerSync:
    def __init__(self, machine, monitor):
        self.machine = machine
        self.monitor = monitor
        
        # Register in __builtin__ context for hooks to find us
        __builtin__.TimerSyncLogic = self

        # Use full names found via GetAllNames
        all_names = list(machine.GetAllNames())
        
        def find_timer(repl_name):
            for n in all_names:
                if n == repl_name or n == "sysbus." + repl_name or n.endswith("." + repl_name):
                    try:
                        return machine[n], n
                    except:
                        pass
            return None, None

        self.timer_objs = {}
        self.timer_names = {}
        self.shadow_regs = {}
        
        for name, repl_name in [("TIM1", "timers1"), ("TIM8", "timers8"), ("TIM20", "timers20")]:
            obj, full_name = find_timer(repl_name)
            self.timer_objs[name] = obj
            self.timer_names[name] = full_name
            self.shadow_regs[name] = {}

        # Register offsets
        self.CR1 = 0x00
        self.CR2 = 0x04
        self.SMCR = 0x08
        self.CNT = 0x24
        self.ARR = 0x2C
        self.CCR4 = 0x40

        # Bit masks for the registers Renode might not hold
        self.BITS_TO_HOLD = {
            self.SMCR: 0x003117CF, # SMS[3:0], TS[4:0], MSM, ETF, ETPS, ETP
            self.CR2:  0x00000070, # MMS[2:0]
        }

        # Follower -> { ITR_Index: Leader_Name }
        self.itr_map = {
            "TIM1": { 3: "TIM8", 8: "TIM20" },
            "TIM8": { 0: "TIM1", 8: "TIM20" },
            "TIM20": { 0: "TIM1", 1: "TIM8" },
        }

        for name, timer in self.timer_objs.items():
            if timer is not None:
                full_name = self.timer_names[name]
                mon_name = full_name.split(".")[-1]
                
                # Use name for Write hook (robust in 1.16)
                hook_code = "TimerSyncLogic.on_write('{}', offset, value)".format(name)
                cmd = "sysbus SetHookBeforePeripheralWrite {} \"{}\"".format(mon_name, hook_code)
                
                try:
                    self.monitor.Parse(cmd)
                    self.machine.Log(LogLevel.Info, "TimerSync: Attached hook to {} ({})".format(name, mon_name))
                except Exception as e:
                    self.machine.Log(LogLevel.Error, "TimerSync: Error attaching hook to {}: {}".format(name, str(e)))
            else:
                self.machine.Log(LogLevel.Warning, "TimerSync: Timer {} NOT FOUND in machine".format(name))

    def on_write(self, name, offset, value):
        old_shadow = self.shadow_regs[name].get(offset, 0)
        
        # Handle Read-Modify-Write (RMW) cycle:
        # If Renode returns 0 on read for unhandled bits, the firmware's RMW will clear them.
        # We restore bits that we want to remain persistent in our shadow state.
        mask = self.BITS_TO_HOLD.get(offset, 0x0)
        if mask != 0:
            # Persistent bits: if the new value clears bits that were in shadow, restore them.
            # However, we only do this if 'value' is a plausible RMW result (e.g. bits we care about are 0).
            # For simplicity, we just keep the bits if they are set in shadow.
            value |= (old_shadow & mask)
        
        self.shadow_regs[name][offset] = value
        
        if offset == self.CR1:
            # Trigger logic when CEN is set (and wasn't before)
            if (value & 1) != 0 and (old_shadow & 1) == 0:
                # Need to use a small delay because other registers might be written 
                # in the same firmware block. But CEN is usually last.
                self.handle_timer_start(name)

    def handle_timer_start(self, name):
        timer = self.timer_objs[name]
        cr2 = self.shadow_regs[name].get(self.CR2, 0)
        mms = (cr2 >> 4) & 0x7
        
        if mms == 7: # OC4REF mode
            ccr4 = self.shadow_regs[name].get(self.CCR4, 0)
            cnt = self.shadow_regs[name].get(self.CNT, 0)
            
            freq = 100000000
            try: freq = timer.Frequency
            except: pass
            if freq == 0: freq = 100000000
            
            tick_ns = 1e9 / freq
            if ccr4 >= cnt:
                delay_ns = int((ccr4 - cnt) * tick_ns)
                self.machine.Log(LogLevel.Info, "TimerSync: Leader {} started. Trigger (CCR4={}) in {}ns".format(name, ccr4, delay_ns))
                
                def make_callback(leader):
                    return lambda *args: self.fire_trigger(leader)
                
                self.machine.ScheduleAction(TimeInterval.FromNanoseconds(delay_ns), make_callback(name))

    def fire_trigger(self, leader_name):
        self.machine.Log(LogLevel.Info, "TimerSync: Triggering from {}".format(leader_name))
        for follower_name, follower_timer in self.timer_objs.items():
            if follower_name == leader_name or follower_timer is None:
                continue
            
            smcr = self.shadow_regs[follower_name].get(self.SMCR, 0)
            sms = (smcr & 0x7) | ((smcr >> 13) & 0x8)
            ts = ((smcr >> 4) & 0x7) | ((smcr >> 17) & 0x18)
            
            if sms == 6: # Trigger Mode
                mapping = self.itr_map.get(follower_name, {})
                if mapping.get(ts) == leader_name:
                    self.machine.Log(LogLevel.Info, "TimerSync: Starting follower {}".format(follower_name))
                    follower_timer.WriteDoubleWord(self.CR1, 1)

# Instance creation
try:
    # When included via .resc, self is the monitor and it has a Machine property.
    # The monitor global is also available.
    ts_obj = TimerSync(self.Machine, monitor)
except Exception as e:
    print("TimerSync init fail: " + str(e))

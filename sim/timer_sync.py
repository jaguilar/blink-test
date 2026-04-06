# Renode script to simulate STM32 Timer TRGO/Slave sync for Renode 1.16.1
from Antmicro.Renode.Logging import Logger, LogLevel
from Antmicro.Renode.Time import TimeInterval

class TimerSync:
    def __init__(self, machine, monitor):
        self.machine = machine
        self.monitor = monitor
        self.sb = machine.SystemBus
        
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
        
        for name, repl_name in [("TIM1", "timers1"), ("TIM8", "timers8"), ("TIM20", "timers20")]:
            obj, full_name = find_timer(repl_name)
            self.timer_objs[name] = obj
            self.timer_names[name] = full_name

        # Follower -> { ITR_Index: Leader_Name }
        self.itr_map = {
            "TIM1": { 5: "TIM8", 9: "TIM20" },
            "TIM8": { 0: "TIM1", 9: "TIM20" },
            "TIM20": { 0: "TIM1", 1: "TIM8" },
        }

        # Register offsets
        self.CR1 = 0x00
        self.CR2 = 0x04
        self.SMCR = 0x08
        self.CCR4 = 0x40
        self.CNT = 0x24

        # Register in global context for hooks
        global TimerSyncLogic
        TimerSyncLogic = self

        for name, timer in self.timer_objs.items():
            if timer is not None:
                full_name = self.timer_names[name]
                # Use Monitor command via monitor.Parse to avoid IronPython generic issues
                hook_code = "TimerSyncLogic.on_write('{}', offset, value)".format(name)
                
                mon_name = full_name
                if mon_name.startswith("sysbus."):
                    mon_name = mon_name[len("sysbus."):]
                
                cmd = "sysbus SetHookBeforePeripheralWrite {} \"{}\"".format(mon_name, hook_code)
                try:
                    self.monitor.Parse(cmd)
                    self.machine.Log(LogLevel.Info, "TimerSync: Attached hook to {} ({})".format(name, mon_name))
                except Exception as e:
                    self.machine.Log(LogLevel.Error, "TimerSync: Error attaching hook to {}: {}".format(name, str(e)))
            else:
                self.machine.Log(LogLevel.Warning, "TimerSync: Timer {} NOT FOUND in machine".format(name))

    def on_write(self, name, offset, value):
        if offset == self.CR1:
            cen = (value & 1) != 0
            if cen:
                timer = self.timer_objs[name]
                cr2 = timer.ReadDoubleWord(self.CR2)
                mms = (cr2 >> 4) & 0x7
                if mms == 7: # OC4REF mode for TRGO
                    ccr4 = timer.ReadDoubleWord(self.CCR4)
                    cnt = timer.ReadDoubleWord(self.CNT)
                    if ccr4 > cnt:
                        delay_ticks = ccr4 - cnt
                        delay_ns = delay_ticks * 10
                        self.machine.Log(LogLevel.Info, "TimerSync: Leader {} starting, scheduling trigger in {}ns".format(name, delay_ns))
                        self.machine.ScheduleAction(TimeInterval.FromNanoseconds(delay_ns), lambda: self.fire_trigger(name))

    def fire_trigger(self, leader_name):
        self.machine.Log(LogLevel.Info, "TimerSync: Firing trigger from leader {}".format(leader_name))
        for follower_name, follower_timer in self.timer_objs.items():
            if follower_name == leader_name or follower_timer is None:
                continue
            smcr = follower_timer.ReadDoubleWord(self.SMCR)
            sms = (smcr & 0x7) | ((smcr >> 13) & 0x8)
            if sms == 6: # Trigger Mode
                ts = ((smcr >> 4) & 0x7) | ((smcr >> 17) & 0x18)
                mapping = self.itr_map.get(follower_name, {})
                if mapping.get(ts) == leader_name:
                    self.machine.Log(LogLevel.Info, "TimerSync: Triggering follower {}".format(follower_name))
                    cr1 = follower_timer.ReadDoubleWord(self.CR1)
                    follower_timer.WriteDoubleWord(self.CR1, cr1 | 1)

# Instance creation
try:
    ts_obj = TimerSync(machine, monitor)
except NameError:
    ts_obj = TimerSync(self.Machine, self)
except Exception as e:
    print("TimerSync initialization failed: " + str(e))

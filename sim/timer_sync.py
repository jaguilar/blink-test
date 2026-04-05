# Renode script to simulate STM32 Timer TRGO/Slave sync for Renode 1.16.1
from Antmicro.Renode.Logging import Logger
from Antmicro.Renode.Time import TimeInterval

class TimerSync:
    def __init__(self, machine):
        self.machine = machine
        self.timers = {
            "TIM1": machine.GetLocalObject("timer1"),
            "TIM8": machine.GetLocalObject("timer8"),
            "TIM20": machine.GetLocalObject("timer20"),
        }
        
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

        for name, timer in self.timers.items():
            if timer:
                # AddPostWriteHook on the CR1 register (offset 0)
                timer.AddPostWriteHook(self.CR1, self.create_on_cr1_write(name))
                self.machine.Log(2, "TimerSync: Attached hook to {}".format(name))

    def create_on_cr1_write(self, name):
        # Create a closure to capture the timer name properly
        return lambda addr, val: self.on_cr1_write(name, val)

    def on_cr1_write(self, name, val):
        cen = (val & 1) != 0
        if cen:
            timer = self.timers[name]
            cr2 = timer.ReadDoubleWord(self.CR2)
            mms = (cr2 >> 4) & 0x7
            if mms == 7: # OC4REF mode for TRGO
                ccr4 = timer.ReadDoubleWord(self.CCR4)
                cnt = timer.ReadDoubleWord(self.CNT)
                if ccr4 > cnt:
                    # Timer in .repl is 100MHz = 10ns per tick
                    delay_ticks = ccr4 - cnt
                    delay_ns = delay_ticks * 10
                    self.machine.Log(2, "TimerSync: Leader {} starting, scheduling trigger in {}ns".format(name, delay_ns))
                    # Schedule an action in virtual time
                    self.machine.StandardControls.ScheduleAction(TimeInterval.FromNanoseconds(delay_ns), lambda: self.fire_trigger(name))

    def fire_trigger(self, leader_name):
        self.machine.Log(2, "TimerSync: Firing trigger from leader {}".format(leader_name))
        for follower_name, follower_timer in self.timers.items():
            if follower_name == leader_name: continue
            smcr = follower_timer.ReadDoubleWord(self.SMCR)
            sms = (smcr & 0x7) | ((smcr >> 13) & 0x8)
            if sms == 6: # Trigger Mode
                ts = ((smcr >> 4) & 0x7) | ((smcr >> 17) & 0x18)
                mapping = self.itr_map.get(follower_name, {})
                if mapping.get(ts) == leader_name:
                    self.machine.Log(2, "TimerSync: Triggering follower {}".format(follower_name))
                    cr1 = follower_timer.ReadDoubleWord(self.CR1)
                    follower_timer.WriteDoubleWord(self.CR1, cr1 | 1)

# Instance creation
# Note: Renode exposes 'machine' in this context
TimerSync(machine)

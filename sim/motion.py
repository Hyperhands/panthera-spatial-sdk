"""Dual host-control boards with a joint-space servo for SDK closed-loop tests."""
import time
from session import SessionBoard


class MotionBoard(SessionBoard):
    def __init__(self,node,rt_port=0,mgmt_port=0):
        super().__init__(node,rt_port,mgmt_port)
        self.last_step=time.monotonic()

    def tick(self,port,cycle):
        now=time.monotonic()
        elapsed=now-self.last_step
        self.last_step=now
        if self.active and self.mit is not None:
            # Gravity-balanced second order position servo. Integrate in small steps
            # so host scheduling jitter changes sampling time, not plant stability.
            while elapsed>0:
                dt=min(elapsed,.002)
                for j in range(6):
                    q=self.arm["q_rad"][j]
                    v=self.arm["dq_rad_s"][j]
                    ref,dq,kp,kd,_=self.mit[5*j:5*j+5]
                    acc=24*kp*(ref-q)+12*kd*(dq-v)-16*v
                    v+=dt*acc
                    self.arm["dq_rad_s"][j]=v
                    self.arm["q_rad"][j]=q+dt*v
                elapsed-=dt
        else:
            self.arm["dq_rad_s"]=[0.]*6
        super().tick(port,cycle)


if __name__=="__main__":
    left,right=MotionBoard(11,15005,15006),MotionBoard(12,15015,15016)
    try:
        cycle=0
        deadline=time.monotonic()
        while True:
            for board,port in ((left,5005),(right,5015)):
                board.service(port,port+1)
                board.tick(port,cycle)
            cycle+=1
            deadline+=.002
            now=time.monotonic()
            if deadline<=now:deadline+=(1+int((now-deadline)/.002))*.002
            time.sleep(max(0,deadline-now))
    except KeyboardInterrupt:
        pass
    finally:
        left.close()
        right.close()

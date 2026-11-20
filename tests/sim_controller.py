from pathlib import Path
import subprocess
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"sim"))
from motion import MotionBoard


def run(exe,mode):
    boards=[MotionBoard(11),MotionBoard(12)]
    for board in boards: board.arm["q_rad"]=[0,1,1.3,-.5,0,0]
    args=[str(p) for b in boards for p in (b.port,b.management.getsockname()[1])]
    with subprocess.Popen([exe,*args,mode],stdout=subprocess.PIPE,text=True) as client:
        try:
            ports=list(map(int,client.stdout.readline().split()))
            cycle=0
            deadline=time.monotonic()
            until=deadline+12
            raw_seen=[False,False]
            injected=False
            holds=[None,None]
            frozen_frames=[0,0]
            while client.poll() is None and time.monotonic()<until:
                for i,b in enumerate(boards):
                    b.service(*ports[2*i:2*i+2])
                    if b.mit and b.mit[::5]==[j+.25 for j in range(6)]: raw_seen[i]=True
                    if i==1 and not injected and mode in ("uplink","reboot","lag","fault"):
                        if all(peer.active and abs(peer.mit[1])>.035 for peer in boards):
                            injected=True
                            if mode=="reboot":b.reset()
                            if mode=="fault":b.arm["fault_code_raw"][2]=1
                    if injected and mode=="lag" and i==1 and b.active:
                        b.arm["q_rad"][0]=b.mit[0]+.07
                    if injected and mode=="uplink" and b.mit and b.mit[1::5]==[0.]*6:
                        current=b.mit[::5]
                        if holds[i] is None:holds[i]=current
                        assert holds[i]==current, "reference advanced after feedback freeze"
                        frozen_frames[i]+=1
                    if not (i==1 and injected and mode=="uplink"):
                        b.tick(ports[2*i],cycle)
                cycle+=1;deadline+=.002
                now=time.monotonic()
                if deadline<=now:deadline+=(1+int((now-deadline)/.002))*.002
                time.sleep(max(0,deadline-now))
            if client.wait(timeout=1): raise RuntimeError(mode)
            assert all(18 in b.operations and not b.active for b in boards)
            if mode=="expert":assert all(raw_seen)
            if mode=="uplink":assert min(frozen_frames)>=2
            if mode in ("uplink","reboot","lag","fault"):assert injected
        finally:
            if client.poll() is None:client.kill()
            for b in boards:b.close()
for scenario in ("step","line","expert","uplink","reboot","lag","fault"):run(sys.argv[1],scenario)

import micropython,utime,machine
from machine import Pin

micropython.alloc_emergency_exception_buf(100)

PIN_OUT=14
PIN_PON=15
BUF_SZ=2048
DEBOUNCE_US=20000
FALSE_MIN_US=60000
FALSE_MAX_US=140000
TRUE_MIN_US=160000
TRUE_MAX_US=260000
TICK59_MIN_US=1200000
REPORT_INTERVAL=8

times=[0]*BUF_SZ
vals=[0]*BUF_SZ
head=0
count=0
last_edge_us=0

pon=Pin(PIN_PON,Pin.OUT)
pon.value(0)
inp=Pin(PIN_OUT,Pin.IN,Pin.PULL_UP)

def irq_handler(pin):
    global head,count,last_edge_us
    now=utime.ticks_us()
    if last_edge_us and utime.ticks_diff(now,last_edge_us) < DEBOUNCE_US:
        return
    last_edge_us=now
    times[head]=now
    vals[head]=pin.value()
    head=(head+1)%BUF_SZ
    if count<BUF_SZ:
        count+=1

inp.irq(trigger=Pin.IRQ_RISING|Pin.IRQ_FALLING,handler=irq_handler)

def snapshot():
    global head,count
    s=machine.disable_irq()
    c=count
    h=head
    if c>BUF_SZ:
        c=BUF_SZ
    arr_t=[0]*c
    arr_v=[0]*c
    start=(h-c)%BUF_SZ
    for i in range(c):
        p=(start+i)%BUF_SZ
        arr_t[i]=times[p]
        arr_v[i]=vals[p]
    machine.enable_irq(s)
    return arr_t,arr_v

def decode_frame(bits):
    if len(bits)!=59:
        return None
    if bits[0]!=0 or bits[20]!=1:
        return None
    def decode_bcd(arr,weights,parity):
        s=0
        pc=0
        for i,w in enumerate(weights):
            if arr[i]:
                s+=w
                pc+=1
        if parity:
            if (pc%2)==0 and arr[-1]:
                return None
        return s
    mins=decode_bcd(bits[21:29],[1,2,4,8,10,20,40,80],True)
    hrs=decode_bcd(bits[29:36],[1,2,4,8,10,20,40],True)
    date_part=bits[36:59]
    day=0
    for i,w in enumerate([1,2,4,8,10,20]):
        if date_part[i]:
            day+=w
    weekday=0
    for i,w in enumerate([1,2,4]):
        if date_part[6+i]:
            weekday+=w
    month=0
    for i,w in enumerate([1,2,4,8,10]):
        if date_part[9+i]:
            month+=w
    year=0
    for i,w in enumerate([1,2,4,8,10,20,40,80]):
        if date_part[14+i]:
            year+=w
    return {'min':mins,'hr':hrs,'day':day,'weekday':weekday,'mon':month,'year':year}

def analyze_and_diagnose():
    tlist,vlist=snapshot()
    n=len(tlist)
    if n<4:
        print("too few edges")
        return
    low_start=None
    low_durs=[]
    low_ts=[]
    rising_times=[]
    for i in range(n-1):
        a=vlist[i]
        b=vlist[i+1]
        ta=tlist[i]
        tb=tlist[i+1]
        if a==1 and b==0:
            low_start=tb
        elif a==0 and b==1 and low_start is not None:
            d=utime.ticks_diff(tb,low_start)
            if d>0:
                low_durs.append(d)
                low_ts.append(low_start)
            low_start=None
            rising_times.append(tb)
    if not low_durs:
        print("no complete low pulses")
        return
    first=tlist[0]; last=tlist[-1]
    duration_s=utime.ticks_diff(last,first)/1_000_000
    pulse_count=len(low_durs)
    avg_us=sum(low_durs)/pulse_count
    short=sum(1 for d in low_durs if FALSE_MIN_US<=d<=FALSE_MAX_US)
    long=sum(1 for d in low_durs if TRUE_MIN_US<=d<=TRUE_MAX_US)
    other=pulse_count-short-long
    rate=pulse_count/duration_s if duration_s>0 else 0
    intersec=[]
    for i in range(len(rising_times)-1):
        intersec.append(utime.ticks_diff(rising_times[i+1],rising_times[i]))
    sec_like=sum(1 for d in intersec if 900000<=d<=1100000)
    tick59_like=sum(1 for d in intersec if d>=TICK59_MIN_US)
    shortlong_ratio=(short+long)/pulse_count
    other_ratio=other/pulse_count
    frames=[]
    for i in range(len(low_ts)-1):
        gap=utime.ticks_diff(low_ts[i+1],low_ts[i])
        if gap>=TICK59_MIN_US:
            start=i+1
            if start+59<=len(low_durs):
                bits=[0 if (FALSE_MIN_US<=low_durs[start+j]<=FALSE_MAX_US) else 1 if (TRUE_MIN_US<=low_durs[start+j]<=TRUE_MAX_US) else None for j in range(59)]
                if None not in bits:
                    dec=decode_frame(bits)
                    if dec:
                        frames.append(dec)
    verdict="uncertain"
    if sec_like>=10 and tick59_like>=1 and shortlong_ratio>=0.6 and other_ratio<=0.3 and frames:
        verdict="likely DCF77"
    elif sec_like<3 and shortlong_ratio<0.5 and other_ratio>0.5:
        verdict="likely NOISE"
    else:
        verdict="uncertain/noisy"
    print("dur:{:.1f}s pulses:{} rate:{:.2f}/s short:{} long:{} other:{} avg_ms:{:.1f} sec_like:{} tick59_like:{} shortlong_ratio:{:.2f} other_ratio:{:.2f} verdict:{}".format(duration_s,pulse_count,rate,short,long,other,avg_us/1000,sec_like,tick59_like,shortlong_ratio,other_ratio,verdict))
    if frames:
        print("decoded frames (sample):",frames[:3])

def main():
    try:
        print("PON low; listening on GPIO",PIN_OUT)
        while True:
            utime.sleep(REPORT_INTERVAL)
            analyze_and_diagnose()
    except KeyboardInterrupt:
        inp.irq(handler=None)
        pon.value(1)
        print("exit")

main()


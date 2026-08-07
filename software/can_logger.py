#!/usr/bin/env python3
"""
can_logger.py  —  CAN + GPS session capture / full simulation mode

Usage:
  pip install pyserial
  python can_logger.py --port COM12 --baud 115200       # real Node A
  python can_logger.py --simulate                        # full fake session
  python can_logger.py --port COM12 --baud 115200 --inject-gps  # real CAN + fake GPS
"""

import argparse, csv, datetime, json, math, os, sys, time, threading, random
try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

WHEELBASE_M  = 2.6
STEER_RATIO  = 14.0

stats = dict(total=0, dec=0, gps=0, uds=0, errors=0, started=time.time())

# ── Real Algiers road waypoints ──────────────────────────────────────────
# Route: Hydra → Bir Mourad Raïs → Ruisseau → back
# Each entry: (lat, lon, heading_deg, speed_kmh, brake_bar)
ALGERIA_ROUTE = [
    (36.737200, 3.086900,  45, 30,  0),
    (36.738100, 3.088200,  52, 35,  0),
    (36.739400, 3.090100,  48, 40,  0),
    (36.740200, 3.092000,  55, 45,  0),
    (36.741000, 3.094300,  60, 50,  2),
    (36.742100, 3.096800,  45, 55,  0),
    (36.743500, 3.098500,  35, 50,  0),
    (36.745000, 3.099600,  15, 45,  5),  # slight brake at junction
    (36.746800, 3.100200,   5, 40,  0),
    (36.748500, 3.100500, 355, 45,  0),
    (36.750000, 3.100100, 345, 50,  0),
    (36.751200, 3.099000, 330, 55,  0),
    (36.752000, 3.097500, 315, 50,  8),  # brake: roundabout
    (36.752500, 3.095800, 300, 35,  0),
    (36.752300, 3.093500, 270, 40,  0),
    (36.751800, 3.091200, 265, 45,  0),
    (36.751000, 3.089100, 255, 50,  0),
    (36.750000, 3.087200, 240, 55,  0),
    (36.748500, 3.085600, 225, 50,  3),
    (36.746800, 3.084200, 220, 45,  0),
    (36.745000, 3.083000, 215, 40,  0),
    (36.743200, 3.082100, 210, 35,  0),
    (36.741500, 3.081500, 200, 30, 10),  # brake: traffic light
    (36.739800, 3.081800, 185, 25,  0),
    (36.738200, 3.082800, 170, 30,  0),
    (36.737200, 3.084200, 145, 35,  0),
    (36.737200, 3.086900,  90, 20, 15),  # arrive back, slow + brake
]

UDS_DIDS = [
    ('0xF190', 'VF14SR6B4FD018433', 28, False),
    ('0xF189', None,                10, False),   # SW version: bytes
    ('0xF18C', 'ECU0123456789',     30, False),
    ('0xF186', '8201312983237R',    35, False),
    ('0xF18E', None,                12, False),   # diag ver: byte
]


# ── Dead reckoning ───────────────────────────────────────────────────────
class DeadReckoning:
    def __init__(self):
        self.x=0.0; self.y=0.0; self.hdg=0.0; self.spd=0.0
        self.last_t=None; self.origin_lat=None; self.origin_lon=None

    def _ll_xy(self, lat, lon):
        R=6_371_000.0
        dlat=(lat-self.origin_lat)*math.pi/180
        dlon=(lon-self.origin_lon)*math.pi/180
        avg=((lat+self.origin_lat)/2)*math.pi/180
        return dlon*R*math.cos(avg), -dlat*R

    def update_gps(self, msg):
        lat,lon,t=msg['lat'],msg['lon'],msg['t']
        if self.origin_lat is None:
            self.origin_lat,self.origin_lon=lat,lon
            self.hdg=msg.get('hdg',0); self.spd=msg.get('spd',0); self.last_t=t
            return None
        x,y=self._ll_xy(lat,lon)
        self.x,self.y=x,y; self.hdg=msg.get('hdg',self.hdg)
        self.spd=msg.get('spd',self.spd); self.last_t=t
        return (t,x,y,self.hdg,self.spd,'GPS')

    def update_can(self, msg):
        if self.last_t is None or self.origin_lat is None: return None
        t=msg['t']; dt=(t-self.last_t)/1000.0
        if dt<=0 or dt>2: self.last_t=t; return None
        wh=msg.get('wh',[0,0,0,0]); spd=sum(wh)/max(len(wh),1)
        steer=msg.get('st',0)/10.0; sms=spd/3.6
        wa=(steer/STEER_RATIO)*math.pi/180; dth=0.0
        if abs(wa)>0.001: dth=(sms*dt)/(WHEELBASE_M/math.tan(wa))
        self.hdg=(self.hdg+math.degrees(dth)+360)%360
        hr=self.hdg*math.pi/180; d=sms*dt
        self.x+=d*math.sin(hr); self.y+=d*math.cos(hr)
        self.spd=spd; self.last_t=t
        return (t,self.x,self.y,self.hdg,self.spd,'DR')


def _stat_printer():
    while True:
        time.sleep(5)
        e=time.time()-stats['started']
        print(f"\r  [{e:5.0f}s] msgs={stats['total']:6d}  "
              f"dec={stats['dec']:5d}  gps={stats['gps']:4d}  "
              f"uds={stats['uds']:3d}  err={stats['errors']:3d}  "
              f"rate={stats['total']/max(e,1):.0f}/s   ",end='',flush=True)


def _open_session(outdir):
    ts=datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    session=os.path.join(outdir,f'session_{ts}')
    os.makedirs(session,exist_ok=True)

    raw_f=open(os.path.join(session,'raw.jsonl'),'w',buffering=1)

    dec_f=open(os.path.join(session,'decoded.csv'),'w',newline='')
    dec_w=csv.writer(dec_f)
    dec_w.writerow(['timestamp_ms','rpm','steer_deg_x10',
                    'wfl','wfr','wrl','wrr','brake_bar','valid_mask'])

    gps_f=open(os.path.join(session,'gps.csv'),'w',newline='')
    gps_w=csv.writer(gps_f)
    gps_w.writerow(['timestamp_ms','lat','lon','speed_kmh',
                    'heading_deg','altitude_m','satellites','hdop'])

    uds_f=open(os.path.join(session,'uds.csv'),'w',newline='')
    uds_w=csv.writer(uds_f)
    uds_w.writerow(['timestamp_ms','did','elapsed_ms',
                    'length','corrupt','data_hex','ascii'])

    dr_f=open(os.path.join(session,'dead_reckoning.csv'),'w',newline='')
    dr_w=csv.writer(dr_f)
    dr_w.writerow(['timestamp_ms','x_m','y_m','heading_deg','speed_kmh','source'])

    return session,(raw_f,dec_f,gps_f,uds_f,dr_f),(dec_w,gps_w,uds_w,dr_w)


def _write_msg(msg, files, writers, dr):
    raw_f,dec_f,gps_f,uds_f,dr_f = files
    dec_w,gps_w,uds_w,dr_w       = writers
    t = msg.get('type','')
    stats['total'] += 1

    raw_f.write(json.dumps(msg)+'\n')

    if t=='dec':
        stats['dec']+=1
        dec_w.writerow([msg.get('t',0),msg.get('rpm',0),msg.get('st',0),
            *msg.get('wh',[0,0,0,0]),msg.get('br',0),msg.get('vm',0)])
        dec_f.flush()
        row=dr.update_can(msg)
        if row: dr_w.writerow([f'{v:.3f}' if isinstance(v,float) else v for v in row]); dr_f.flush()

    elif t=='dr':
        # ESP32 already computed dead reckoning — just save it directly
        dr_w.writerow([msg.get('t',0),
            f"{msg.get('x',0):.3f}", f"{msg.get('y',0):.3f}",
            f"{msg.get('hdg',0):.1f}", f"{msg.get('spd',0):.1f}",
            msg.get('gc',0)])
        dr_f.flush()

    elif t=='gps':
        stats['gps']+=1
        gps_w.writerow([msg.get('t',0),msg.get('lat',0),msg.get('lon',0),
            msg.get('spd',0),msg.get('hdg',0),msg.get('alt',0),
            msg.get('sat',0),msg.get('hdop',0)])
        gps_f.flush()
        row=dr.update_gps(msg)
        if row: dr_w.writerow([f'{v:.3f}' if isinstance(v,float) else v for v in row]); dr_f.flush()

    elif t=='uds':
        stats['uds']+=1
        uds_w.writerow([msg.get('t',0),msg.get('did',''),msg.get('ms',0),
            msg.get('len',0),msg.get('corrupt',False),msg.get('d',''),msg.get('ascii','')])
        uds_f.flush()


# ── Simulation generator ─────────────────────────────────────────────────
def run_simulate(session, files, writers, dr, duration_s=300):
    """
    Generate a full fake session: CAN signals + GPS along the Algiers route.
    duration_s: how many seconds of simulated driving to produce.
    """
    print(f'\nSimulating {duration_s}s drive on Algiers streets…')
    print('Route: Hydra → Bir Mourad Raïs → Ruisseau → back\n')

    n   = len(ALGERIA_ROUTE)
    t0  = int(time.time()*1000)
    uds_idx    = 0
    uds_next_t = t0 + 3000   # first UDS DID at t=3s

    # How many ticks total at 20Hz
    total_ticks = duration_s * 20
    seg_ticks   = total_ticks / n   # ticks per route segment

    for tick in range(total_ticks):
        now_ms = t0 + tick * 50   # 50ms per tick = 20Hz

        # ── Interpolate position along route ──────────────────
        seg   = int(tick / seg_ticks) % n
        frac  = (tick % seg_ticks) / seg_ticks
        a     = ALGERIA_ROUTE[seg]
        b     = ALGERIA_ROUTE[(seg+1) % n]

        lat   = a[0] + (b[0]-a[0])*frac
        lon   = a[1] + (b[1]-a[1])*frac
        hdg   = a[2] + (b[2]-a[2])*frac
        spd   = a[3] + (b[3]-a[3])*frac + random.gauss(0,1)
        br    = max(0, round(a[4] + (b[4]-a[4])*frac + random.gauss(0,0.5)))
        spd   = max(0, spd)

        # ── CAN decoded frame (every tick = 20Hz) ─────────────
        steer = round((hdg - ALGERIA_ROUTE[0][2]) * 8 + random.gauss(0,5)) * 10
        rpm   = round(800 + spd*55 + random.gauss(0,50))
        wh    = [round(spd+random.gauss(0,.3)) for _ in range(4)]
        wh[2] = max(0,wh[2]-1)  # rear-left slightly less (normal)

        dec_msg = {
            't':  now_ms,
            'type':'dec',
            'rpm': max(800,rpm),
            'st':  steer,
            'wh':  wh,
            'br':  br,
            'vm':  15,
        }
        _write_msg(dec_msg, files, writers, dr)

        # ── GPS fix (every 10 ticks = 2Hz, simulates 10Hz NEO-6M) ──
        if tick % 5 == 0:
            # Add small GNSS noise (±2m)
            noise_lat = random.gauss(0, 0.00002)
            noise_lon = random.gauss(0, 0.00002)
            gps_msg = {
                't':    now_ms,
                'type': 'gps',
                'lat':  lat + noise_lat,
                'lon':  lon + noise_lon,
                'spd':  round(spd*10)/10,
                'hdg':  round(hdg*10)/10,
                'alt':  210.0 + random.gauss(0,1),
                'sat':  random.choice([9,9,9,10,10,11]),
                'hdop': round(random.uniform(0.9,1.4),1),
                'fix':  tick//5 + 1,
            }
            _write_msg(gps_msg, files, writers, dr)

        # ── UDS DIDs (one every ~5s) ───────────────────────────
        if now_ms >= uds_next_t:
            did, ascii_val, ms, corrupt = UDS_DIDS[uds_idx % len(UDS_DIDS)]
            uds_idx += 1
            uds_next_t = now_ms + 5000 + random.randint(0,2000)
            if ascii_val:
                d = ' '.join(f'{ord(c):02X}' for c in ascii_val[:8])
            else:
                d = '00 2C' if did=='0xF189' else '84'
            uds_msg = {
                't':      now_ms,
                'type':   'uds',
                'did':    did,
                'ms':     ms + random.randint(-3,10),
                'len':    len(ascii_val) if ascii_val else 1,
                'corrupt': corrupt,
                'd':      d,
                'ascii':  ascii_val or '',
            }
            _write_msg(uds_msg, files, writers, dr)

        # ── Progress bar ───────────────────────────────────────
        if tick % 200 == 0:
            pct = tick/total_ticks*100
            bar = '█'*int(pct/4)+' '*(25-int(pct/4))
            elapsed = tick*50/1000
            print(f'\r  [{bar}] {pct:5.1f}%  t={elapsed:.0f}s  '
                  f'lat={lat:.5f}  spd={spd:.0f}km/h  '
                  f'msgs={stats["total"]}   ',end='',flush=True)

    print(f'\n\nSimulation complete.')


# ── Real serial mode ─────────────────────────────────────────────────────
def run_serial(port, baud, inject_gps, session, files, writers, dr):
    if not HAS_SERIAL:
        print('ERROR: pyserial not installed. Run: pip install pyserial')
        sys.exit(1)

    # GPS injector state (for --inject-gps mode)
    gps_state = {'seg':0,'frac':0.0,'tick':0}

    def gps_injector():
        """Injects fake GPS into the message stream when --inject-gps is set."""
        t0=int(time.time()*1000)
        seg=0; frac=0.0; n=len(ALGERIA_ROUTE)
        while True:
            time.sleep(0.1)   # 10Hz
            frac+=0.006
            if frac>=1: frac=0; seg=(seg+1)%n
            a=ALGERIA_ROUTE[seg]; b=ALGERIA_ROUTE[(seg+1)%n]
            msg={'t':int(time.time()*1000),'type':'gps',
                 'lat':a[0]+(b[0]-a[0])*frac + random.gauss(0,0.00002),
                 'lon':a[1]+(b[1]-a[1])*frac + random.gauss(0,0.00002),
                 'spd':a[3]+(b[3]-a[3])*frac,
                 'hdg':a[2]+(b[2]-a[2])*frac,
                 'alt':210.0,'sat':10,'hdop':1.1,'fix':1}
            _write_msg(msg,files,writers,dr)

    if inject_gps:
        threading.Thread(target=gps_injector,daemon=True).start()
        print('GPS injection active — simulating Algiers route')

    print(f'Connecting to {port} @ {baud}…')
    try:
        ser=serial.Serial(port,baud,timeout=1)
        print('Connected. Ctrl+C to stop.\n')
    except serial.SerialException as e:
        print(f'ERROR: {e}'); sys.exit(1)

    buf=''
    try:
        while True:
            try:
                chunk=ser.read(ser.in_waiting or 1).decode('utf-8',errors='ignore')
                buf+=chunk
                while '\n' in buf:
                    line,buf=buf.split('\n',1)
                    line=line.strip()
                    if not line.startswith('{'): continue
                    try:
                        m=json.loads(line)
                        _write_msg(m,files,writers,dr)
                    except json.JSONDecodeError:
                        stats['errors']+=1
            except Exception as e:
                if 'disconnected' in str(e).lower(): break
                stats['errors']+=1
    except KeyboardInterrupt:
        print('\n\nStopped.')
    finally:
        ser.close()


# ── Main ─────────────────────────────────────────────────────────────────
def main():
    ap=argparse.ArgumentParser(description='CAN+GPS session logger / simulator')
    ap.add_argument('--port',       default='COM12',  help='Serial port')
    ap.add_argument('--baud',       type=int,default=115200)
    ap.add_argument('--outdir',     default='logs')
    ap.add_argument('--simulate',   action='store_true',
                    help='Generate full fake session (no hardware needed)')
    ap.add_argument('--duration',   type=int,default=300,
                    help='Simulation duration in seconds (default 300)')
    ap.add_argument('--inject-gps', action='store_true',
                    help='Real CAN from serial + fake GPS injected alongside')
    args=ap.parse_args()

    session,files,writers=_open_session(args.outdir)
    raw_f,dec_f,gps_f,uds_f,dr_f=files

    print(f'Session: {session}')
    print(f'  raw.jsonl  decoded.csv  gps.csv  uds.csv  dead_reckoning.csv')

    dr=DeadReckoning()
    threading.Thread(target=_stat_printer,daemon=True).start()

    try:
        if args.simulate:
            run_simulate(session,files,writers,dr,args.duration)
        else:
            run_serial(args.port,args.baud,args.inject_gps,session,files,writers,dr)
    finally:
        for f in files: f.close()
        e=time.time()-stats['started']
        print(f'\nSummary: {e:.0f}s  {stats["total"]} msgs  '
              f'{stats["dec"]} dec  {stats["gps"]} gps  {stats["uds"]} uds')
        print(f'Saved: {session}/')

if __name__=='__main__':
    main()
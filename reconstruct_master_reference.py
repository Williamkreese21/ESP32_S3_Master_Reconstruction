import re, math, uuid, os
from collections import defaultdict
from shapely.geometry import LineString, Point

from pathlib import Path
ROOT=Path(__file__).resolve().parent
SRC=str(ROOT/'source_reference/ESP32_S3_Modular_Carrier_REV4.kicad_pcb')
OUT=str(ROOT/'ESP32_S3_Master_Reconstruction.kicad_pcb')

src=open(SRC,encoding='utf-8').read()

def blocks(text, token):
    for m in re.finditer(re.escape(token), text):
        st=m.start(); depth=0
        for i in range(st,len(text)):
            if text[i]=='(': depth+=1
            elif text[i]==')':
                depth-=1
                if depth==0:
                    yield text[st:i+1]; break

def parse_footprints(board):
    fps={}
    for fb in blocks(board,'(footprint "'):
        fm=re.match(r'\(footprint "([^"]+)"',fb)
        if not fm: continue
        name=fm.group(1)
        am=re.search(r'\(at\s+([-\d.]+)\s+([-\d.]+)(?:\s+([-\d.]+))?\)',fb)
        if not am: continue
        fx,fy,fr=map(float,am.groups(default='0'))
        pads=[]
        for pb in blocks(fb,'(pad "'):
            pm=re.match(r'\(pad "([^"]+)"',pb)
            at=re.search(r'\(at\s+([-\d.]+)\s+([-\d.]+)(?:\s+([-\d.]+))?\)',pb)
            nm=re.search(r'\(net\s+"([^"]+)"\)',pb)
            sz=re.search(r'\(size\s+([-\d.]+)\s+([-\d.]+)\)',pb)
            dr=re.search(r'\(drill\s+([-\d.]+)',pb)
            if not (pm and at and nm): continue
            dx,dy,r=map(float,at.groups(default='0'))
            a=math.radians(fr)
            x=fx+dx*math.cos(a)-dy*math.sin(a); y=fy+dx*math.sin(a)+dy*math.cos(a)
            pads.append(dict(pad=pm.group(1),x=x,y=y,net=nm.group(1),size=float(sz.group(1)) if sz else 2.0,drill=float(dr.group(1)) if dr else 0.9))
        fps[name]=pads
    return fps

fps=parse_footprints(src)
net_names=[]
for pads in fps.values():
    for p in pads:
        if p['net'] not in net_names: net_names.append(p['net'])
net_id={n:i+1 for i,n in enumerate(net_names)}

def P(fp,pad):
    for p in fps[fp]:
        if p['pad']==str(pad): return (p['x'],p['y'])
    raise KeyError((fp,pad))

routes=[]
def add(net,layer,pts,width=0.32):
    pts=[(round(float(x),3),round(float(y),3)) for x,y in pts]
    q=[]
    for pt in pts:
        if not q or pt!=q[-1]: q.append(pt)
    routes.append((net,layer,q,width))

# Produce a path with only 45-degree diagonals and axis-aligned segments,
# with two 45-degree corners. The dominant axis gets the middle straight segment.
def dogleg(a,b,small=6.0):
    x1,y1=a; x2,y2=b
    dx=x2-x1; dy=y2-y1
    sx=1 if dx>=0 else -1; sy=1 if dy>=0 else -1
    adx=abs(dx); ady=abs(dy)
    if adx < 1e-6 or ady < 1e-6:
        # avoid a 90-degree corner by using a tiny 45-degree chamfer at one end if possible
        if adx < 1e-6 and ady < 1e-6: return [a,b]
        if adx < 1e-6:
            t=min(4.0, ady/2)
            return [a,(x1+sx*t,y1+sy*t),(x2,y2)]
        t=min(4.0, adx/2)
        return [a,(x1+sx*t,y1+sy*t),(x2,y2)]
    if adx >= ady:
        # diag length split; middle horizontal
        t=min(max(3.0,small), ady*0.45)
        u=ady-t
        p1=(x1+sx*u, y1+sy*u)
        p2=(x2-sx*t, y2-sy*t)
        return [a,p1,p2,b]
    else:
        t=min(max(3.0,small), adx*0.45)
        u=adx-t
        p1=(x1+sx*u,y1+sy*u)
        p2=(x2-sx*t,y2-sy*t)
        return [a,p1,p2,b]

# --- Shared RF SPI/data bundles on B.Cu, routed across the top in two parallel trunks ---
# GPIO6 shared between J1, RF1 pad5, RF2 pad5
C6=(61.3,26.5)
add('GPIO6','B.Cu',dogleg(P('Custom:J1',6),C6,6.0))
add('GPIO6','B.Cu',dogleg(C6,P('Custom:J_RF1',5),6.0))
add('GPIO6','B.Cu',dogleg(C6,P('Custom:J_RF2',5),6.0))
# GPIO7 shared between J1, RF1 pad6, RF2 pad6
C7=(61.3,31.0)
add('GPIO7','B.Cu',dogleg(P('Custom:J1',7),C7,6.0))
add('GPIO7','B.Cu',dogleg(C7,P('Custom:J_RF1',6),6.0))
add('GPIO7','B.Cu',dogleg(C7,P('Custom:J_RF2',6),6.0))

# --- Left RF connector bundle on F.Cu ---
add('GPIO4','F.Cu',dogleg(P('Custom:J1',4),P('Custom:J_RF1',3),7.0))
add('GPIO5','F.Cu',dogleg(P('Custom:J1',5),P('Custom:J_RF1',7),7.0))
add('GPIO3','F.Cu',dogleg(P('Custom:J1',13),P('Custom:J_RF1',4),8.0))

# --- Right RF connector bundle on F.Cu ---
add('GPIO1','F.Cu',dogleg(P('Custom:J3',4),P('Custom:J_RF2',4),7.0))
add('GPIO2','F.Cu',dogleg(P('Custom:J3',5),P('Custom:J_RF2',3),7.0))
add('GPIO5','F.Cu',dogleg(P('Custom:J3',6-1),P('Custom:J_RF2',7),7.0))  # J3 pad5 -> GPIO2 actually, corrected below
# Replace the accidental line above by removing last route and add correct GPIO5 route from J1 shared pad
routes.pop()
add('GPIO5','B.Cu',dogleg(P('Custom:J1',5),P('Custom:J_RF2',7),8.0))

# --- TFT 1x8 bus, F.Cu, monotonic fan-out ---
for net,jp,tp in [
    ('GPIO8',12,7),('GPIO10',16,6),('GPIO9',15,5),('GPIO11',17,4),('GPIO12',18,3)
]:
    add(net,'F.Cu',dogleg(P('Custom:J1',jp),P('Custom:J_TFT_BUS',tp),5.0))

# --- GPS UART pair ---
add('GPIO18','F.Cu',dogleg(P('Custom:J3',11),P('Custom:J_GPS1',3),6.0))
add('GPIO17','F.Cu',dogleg(P('Custom:J3',10),P('Custom:J_GPS1',4),6.0))

# --- Four buttons, F.Cu; sources are kept on dedicated radial corridors ---
add('GPIO13','F.Cu',dogleg(P('Custom:J1',19),P('Custom:SW1',1),4.0))
add('GPIO13','F.Cu',[(P('Custom:SW1',1)),P('Custom:SW1',2)])
add('GPIO14','F.Cu',dogleg(P('Custom:J1',20),P('Custom:SW2',1),4.0))
add('GPIO14','F.Cu',[P('Custom:SW2',1),P('Custom:SW2',2)])
add('GPIO15','F.Cu',dogleg(P('Custom:J1',8),P('Custom:SW3',1),7.0))
add('GPIO15','F.Cu',[P('Custom:SW3',1),P('Custom:SW3',2)])
add('GPIO16','F.Cu',dogleg(P('Custom:J1',9),P('Custom:SW4',1),7.0))
add('GPIO16','F.Cu',[P('Custom:SW4',1),P('Custom:SW4',2)])

# --- Encoder on B.Cu, using three lower parallel corridors ---
for net,jp,ex,ep in [
    ('GPIO41',7,103,5),('GPIO40',8,99,4),('GPIO39',9,95,3)
]:
    a=P('Custom:J3',jp); target=P('Custom:J_ENC1',ep)
    # force a safe B.Cu lane before the lower horizontal fan-out
    lane=(ex,104.0)
    add(net,'B.Cu',dogleg(a,lane,7.0))
    add(net,'B.Cu',dogleg(lane,target,7.0))

# --- Buzzer and GPIO42 path, F.Cu outer-right corridor ---
add('GPIO42','F.Cu',dogleg(P('Custom:J3',6),P('Custom:R5',1),8.0))
add('BUZZER_LOW','F.Cu',[(P('Custom:BZ1',2)),(165.62,112),(161.81,123)])
# GPIO42 enters R5; R5 -> Q1 base, R6 -> Q1 base
add('BUZZER_BASE','F.Cu',[(P('Custom:R5',2)),(152.54,123),(155.46,123)])
add('BUZZER_BASE','F.Cu',[(P('Custom:R6',1)),(145.0,124.46),(147.54,123)])

# --- Power chain, B.Cu. Protected 5V is kept above encoder corridors. ---
add('5V_IN','B.Cu',dogleg(P('Custom:J_PWR',1),P('Custom:F1',1),4.0))
add('5V_FUSED','B.Cu',dogleg(P('Custom:F1',2),P('Custom:D1',1),4.0))
# 5V_PROTECTED from D1 to J1 pin21 using a dedicated y=98 corridor, staying right of encoder lanes.
add('5V_PROTECTED','B.Cu',[
    P('Custom:D1',2),(150.0,118.46),(150.0,98.0),(120.0,98.0),(112.8,90.8),P('Custom:J1',21)
])

# --- Battery sense, B.Cu right-side compact analog loop ---
add('VBAT_RAW','B.Cu',dogleg(P('Custom:J_BAT',1),P('Custom:R13',1),3.5))
add('BAT_ADC','B.Cu',[(P('Custom:R13',2)),(168.5,94),(168.5,99),(166.54,100)])
add('BAT_ADC','B.Cu',[(P('Custom:R14',1)),(171.0,100),(171.0,99),(173.46,99)])

# --- GND is a B.Cu copper zone; no unnecessary ground tracks ---

# ---------- Root net declarations / pad net IDs ----------
netdecl='\t(net 0 "")\n'+''.join(f'\t(net {nid} "{name}")\n' for name,nid in net_id.items())
le=src.find('\t(setup')
if le<0: raise RuntimeError('setup not found')
board=src[:le]+netdecl+src[le:]
for name,nid in net_id.items():
    board=board.replace(f'(net "{name}")',f'(net {nid} "{name}")')

# ---------- Graphics ----------
g=[]
def gid(): return str(uuid.uuid4())
def grtext(text,x,y,rot=0,size=1.2,thick=0.18):
    # encode line breaks as literal KiCad escape sequence
    text=text.replace('\\','\\\\').replace('"','\\"')
    g.append(f'''\t(gr_text "{text}"\n\t\t(at {x} {y} {rot})\n\t\t(layer "F.SilkS")\n\t\t(effects (font (size {size} {size}) (thickness {thick})))\n\t\t(uuid "{gid()}")\n\t)''')
def grrect(x1,y1,x2,y2,w=0.25):
    g.append(f'''\t(gr_rect\n\t\t(start {x1} {y1})\n\t\t(end {x2} {y2})\n\t\t(stroke (width {w}) (type default))\n\t\t(fill none)\n\t\t(layer "F.SilkS")\n\t\t(uuid "{gid()}")\n\t)''')

grtext('ESP32-S3 N16R8 MODULAR CARRIER',100,15.5,0,1.65,0.28)
grtext('REV4.1  •  2-LAYER  •  SOCKETED DEVKITC-1',100,18.5,0,0.85,0.14)
grrect(24,12,46,23,0.3); grrect(144,12,166,23,0.3)
grtext('ANTENNA KEEP-OUT',35,17.2,0,1.0,0.16); grtext('ANTENNA KEEP-OUT',155,17.2,0,1.0,0.16)
# central module silhouette
# keep text away from the actual board edges
grrect(63.5,28,126.5,108,0.35); grrect(67,31.5,123,105,0.18)
grtext('ESP32-S3 DEVKITC-1',95,69,90,1.55,0.24)
grtext('SOCKETED / REMOVABLE',95,89,90,0.85,0.15)
# peripheral labels
for t,x,y,r in [
    ('RF1',35,26.7,0),('nRF24L01+',35,24.8,0),('RF2',155,26.7,0),('nRF24L01+',155,24.8,0),
    ('TFT BUS 1x8',35,106.6,0),('NEO-6M / UART',155,104.5,0),('KY-040',30,118,0),
    ('UP',86,119.2,0),('DOWN',104,119.2,0),('ENTER',122,119.2,0),('BACK',140,119.2,0),
    ('BUZZER',158,106.5,0),('PWR',176,118,90),('BAT',176,84,90),('REV4.1',17.5,70,90),
]: grtext(t,x,y,r,1.0,0.17)
# silkscreen pin labels around the two ESP32 headers (compact, readable)
left_labels=['3V3','3V3','RST','GPIO4','GPIO5','GPIO6','GPIO7','GPIO15','GPIO16','GPIO17','GPIO18','GPIO8','GPIO3','GPIO46','GPIO9','GPIO10','GPIO11','GPIO12','GPIO13','GPIO14','5V','GND']
right_labels=['GND','GPIO43','GPIO44','GPIO1','GPIO2','GPIO42','GPIO41','GPIO40','GPIO39','GPIO38','GPIO37','GPIO36','GPIO35','GPIO0','GPIO45','GPIO48','GPIO47','GPIO21','GPIO20','GPIO19','GND','GND']
for i,t in enumerate(left_labels): grtext(t,78.8,40+i*2.54,0,0.55,0.10)
for i,t in enumerate(right_labels): grtext(t,111.2,40+i*2.54,0,0.55,0.10)

# segments
segments=[]
for net,layer,pts,width in routes:
    nid=net_id[net]
    for a,b in zip(pts,pts[1:]):
        if a==b: continue
        segments.append(f'''\t(segment\n\t\t(start {a[0]} {a[1]})\n\t\t(end {b[0]} {b[1]})\n\t\t(width {width})\n\t\t(layer "{layer}")\n\t\t(net {nid})\n\t\t(uuid "{gid()}")\n\t)''')

# Ground plane B.Cu. The top 13 mm remain solid-free for RF antenna keep-out.
zone=f'''\t(zone\n\t\t(net {net_id['GND']})\n\t\t(net_name "GND")\n\t\t(layer "B.Cu")\n\t\t(hatch edge 0.5)\n\t\t(connect_pads (clearance 0.3))\n\t\t(min_thickness 0.25)\n\t\t(fill yes (thermal_gap 0.3) (thermal_bridge_width 0.3))\n\t\t(polygon\n\t\t\t(pts\n\t\t\t\t(xy 13 26) (xy 177 26) (xy 177 127) (xy 13 127)\n\t\t\t)\n\t\t)\n\t\t(uuid "{gid()}")\n\t)'''

# 3V3 plane on F.Cu: entire useful board area, kept clear of the two antenna boxes.
zone33=f'''\t(zone\n\t\t(net {net_id['3V3']})\n\t\t(net_name "3V3")\n\t\t(layer "F.Cu")\n\t\t(hatch edge 0.5)\n\t\t(connect_pads (clearance 0.3))\n\t\t(min_thickness 0.25)\n\t\t(fill yes (thermal_gap 0.3) (thermal_bridge_width 0.3))\n\t\t(polygon\n\t\t\t(pts\n\t\t\t\t(xy 13 26) (xy 177 26) (xy 177 127) (xy 13 127)\n\t\t\t)\n\t\t)\n\t\t(uuid "{gid()}")\n\t)'''

insert='\n'.join(g+segments+[zone,zone33])+'\n'
idx=board.rfind('\n)')
board=board[:idx]+'\n'+insert+board[idx:]
board=board.replace('(title "ESP32-S3 N16R8 Modular Carrier - REV4")','(title "ESP32-S3 N16R8 Modular Carrier - REV4.1 PROFESSIONAL ROUTE")')
open(OUT,'w',encoding='utf-8').write(board)

# ---------- geometry audit ----------
# exact turn-angle audit
bad_turns=[]
for rid,(net,layer,pts,w) in enumerate(routes):
    for a,b,c in zip(pts,pts[1:],pts[2:]):
        v1=(b[0]-a[0],b[1]-a[1]); v2=(c[0]-b[0],c[1]-b[1])
        l1=math.hypot(*v1); l2=math.hypot(*v2)
        if l1<1e-9 or l2<1e-9: continue
        a1=math.degrees(math.atan2(v1[1],v1[0]))%360; a2=math.degrees(math.atan2(v2[1],v2[0]))%360
        d=abs(a2-a1); d=min(d,360-d)
        if min(abs(d-45),abs(d),abs(d-180))>1e-3:
            bad_turns.append((rid,net,layer,a,b,c,d))

# same-layer different-net crossings and very-close centerlines
segs=[]
for rid,(net,layer,pts,w) in enumerate(routes):
    for j,(a,b) in enumerate(zip(pts,pts[1:])):
        if a==b: continue
        segs.append((rid,j,net,layer,w,LineString([a,b])))
cross=[]; near=[]
for i in range(len(segs)):
    for j in range(i+1,len(segs)):
        A=segs[i]; B=segs[j]
        if A[3]!=B[3] or A[2]==B[2]: continue
        d=A[5].distance(B[5])
        if d<0.35: near.append((d,A,B))
        if d<1e-6: cross.append((A,B))

report=f'''Professional PCB layout audit\n============================\nOutput: {OUT}\nNets: {len(net_id)}\nRoutes: {len(routes)}\nSegments: {len(segments)}\nVias: 0 (no signal-layer changes required)\nTurn violations (anything other than 0/45/180): {len(bad_turns)}\nDifferent-net same-layer centerline crossings: {len(cross)}\nDifferent-net same-layer centerline spacing <0.35 mm: {len(near)}\nNote: KiCad GUI/kicad-cli DRC was not available in this runtime; audit is geometric and syntax-based.\n'''
open('/mnt/data/ESP32_S3_Modular_Carrier_REV4_PRO_LAYOUT_AUDIT.txt','w').write(report)
print(report)
print('first bad turns',bad_turns[:8])
print('first crossings',[(a[2],a[3],b[2],b[3]) for a,b in cross[:15]])
print('first near',[(round(d,3),a[2],b[2],a[3]) for d,a,b in near[:15]])

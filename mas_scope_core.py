"""MAS Scope Tool - ядро: чтение/запись OGF (skinned v4, X-Ray) и приведение прицела к правилам MAS.

Правила (одинаковые для всех прицелов):
  1. размер - как у этого прицела в Pizza's Ultimate Sight (эталон ищется по текстуре корпуса);
     нет эталона - масштаб по умолчанию из настроек;
  2. по центру крепление (середина между зажимами) на x = 0, а не сам прицел;
  3. все крепления на одной высоте: низ крепления на y = clamp_bottom (как у acog/ado, около -5 мм);
  4. задний край крепления на z = 0, всё крепление впереди центра (корпус/окуляр могут выходить назад);
  5. шейдеры *_3db -> обычные из Pizza; кости wpn_scope (+ lens, если на ней есть геометрия), бинд единичный;
     анимации attachments\\motions\\common или common_lens;
  6. aim_hud_offset = aim эталона - (сетка новая - сетка эталона); без эталона - по оси сетки и aim z по умолчанию.
"""
import os, re, struct
import numpy as np

# ---------------------------------------------------------------- OGF
VF = {1: ('1L', 1), 2: ('2L', 2), 3: ('3L', 3), 4: ('4L', 4),
      0x12071980: ('1L', 1), 0x240E3300: ('2L', 2), 0x481C6600: ('3L', 3), 0x5A23BF80: ('4L', 4)}
STRIDE = {'1L': 60, '2L': 64, '3L': 70, '4L': 76}

def chunks(b):
    out, p = [], 0
    while p + 8 <= len(b):
        cid, sz = struct.unpack_from('<II', b, p)
        out.append([cid, bytes(b[p + 8:p + 8 + sz])])
        p += 8 + sz
    return out

def pack(chs):
    return b''.join(struct.pack('<II', c, len(d)) + d for c, d in chs)

def cstr(b, p):
    e = b.index(b'\0', p)
    return b[p:e].decode('cp1251', 'replace'), e + 1

def read_verts(d):
    fmt, n = struct.unpack_from('<II', d, 0)
    kind, nb = VF[fmt]
    s = STRIDE[kind]
    raw = np.frombuffer(d, np.uint8, n * s, 8).reshape(n, s)
    if kind == '1L':
        P = raw[:, 0:12].copy().view('<f4').reshape(n, 3)
        B = raw[:, 56:60].copy().view('<u4').reshape(n, 1).astype(int)
    else:
        B = raw[:, 0:2 * nb].copy().view('<u2').reshape(n, nb).astype(int)
        P = raw[:, 2 * nb:2 * nb + 12].copy().view('<f4').reshape(n, 3)
    return kind, P.astype(np.float64), B

def write_verts(d, P, B):
    fmt, n = struct.unpack_from('<II', d, 0)
    kind, nb = VF[fmt]
    s = STRIDE[kind]
    b = bytearray(d)
    raw = np.frombuffer(b, np.uint8, n * s, 8).reshape(n, s)
    if kind == '1L':
        raw[:, 0:12] = P.astype('<f4').view(np.uint8).reshape(n, 12)
        raw[:, 56:60] = B[:, :1].astype('<u4').view(np.uint8).reshape(n, 4)
    else:
        raw[:, 0:2 * nb] = B.astype('<u2').view(np.uint8).reshape(n, 2 * nb)
        raw[:, 2 * nb:2 * nb + 12] = P.astype('<f4').view(np.uint8).reshape(n, 12)
    return bytes(b)

def read_bone_names(d):
    n = struct.unpack_from('<I', d, 0)[0]
    p, out = 4, []
    for _ in range(n):
        name, p = cstr(d, p)
        par, p = cstr(d, p)
        out.append([name, par, list(struct.unpack_from('<15f', d, p))])
        p += 60
    return out

def read_ik(d, n):
    out, p = [], 0
    for _ in range(n):
        s = p
        p += 4
        p = d.index(b'\0', p) + 1
        p += 4 + 108 + 4 + 48 + 24 + 40
        out.append(bytearray(d[s:p]))
    return out

def header(h, P):
    lo, hi = P.min(0), P.max(0)
    c = (lo + hi) / 2
    return h[:4] + struct.pack('<6f4f', *lo, *hi, *c, float(np.linalg.norm(P - c, axis=1).max())) + h[44:]

SHADER_FIX = {'models\\reflex_reticle_3db': 'models\\reflex_reticle',
              'models\\reflex_reticle_simple_3db': 'models\\reflex_reticle_simple'}

class Model:
    def __init__(self, path):
        self.path = path
        self.top = chunks(open(path, 'rb').read())
        T = dict(self.top)
        self.T = T
        self.kids = []
        for _, d in chunks(T[9]):
            k = dict(chunks(d))
            tex, q = cstr(k[2], 0)
            sh, _ = cstr(k[2], q)
            kind, P, B = read_verts(k[3])
            self.kids.append(dict(raw=k, tex=tex, sh=sh, P=P, B=B))
        self.bones = read_bone_names(T[13]) if 13 in T else [['wpn_scope', '', [0.0] * 15]]

    def all_points(self):
        return np.vstack([k['P'] for k in self.kids])

    def body(self):
        m = [k for k in self.kids if 'reticle' not in k['sh'] and 'lens' not in k['sh'] and k['sh'] not in ('models\\scope_back', 'models\\scope_zwrite')]
        return max(m or self.kids, key=lambda k: len(k['P']))

    def reticle(self):
        """Прицельная марка: задняя (у оптики) из плоских марок; центр и размер."""
        r = [k for k in self.kids if 'reticle' in k['sh']]
        if not r:
            return None, None
        r.sort(key=lambda k: k['P'][:, 2].mean())
        P = r[0]['P']
        return (P.min(0) + P.max(0)) / 2, np.ptp(P[:, :2], 0)

def tex_key(t):
    return os.path.basename(t.replace('\\', '/')).lower()

# ---------------------------------------------------------------- эталоны Pizza
class PizzaRefs:
    """Модели attachments\\scopes из папки мода Pizza и их aim_hud_offset из конфигов."""
    def __init__(self, root):
        self.root = root
        self.models = {}   # name -> dict(path, tex set, body_tex, body_n, body_ptp, ret, ret_size)
        self.aim = {}      # name -> (pos, alt_pos or None)
        self.error = ''
        if not root:
            return
        mdir = os.path.join(root, 'gamedata', 'meshes', 'attachments', 'scopes')
        if not os.path.isdir(mdir):
            self.error = 'нет папки ' + mdir
            return
        for f in sorted(os.listdir(mdir)):
            if not f.lower().endswith('.ogf'):
                continue
            try:
                m = Model(os.path.join(mdir, f))
            except Exception:
                continue
            b = m.body()
            ret, rsz = m.reticle()
            self.models[f[:-4].lower()] = dict(path=m.path, tex={tex_key(k['tex']) for k in m.kids}, body_tex=tex_key(b['tex']),
                                              body_P=b['P'], ret=ret, ret_size=rsz)
        self._read_configs(os.path.join(root, 'gamedata', 'configs'))

    def _read_configs(self, cdir):
        files = []
        for dp, _, fs in os.walk(cdir):
            for f in fs:
                if f.lower().endswith('.ltx'):
                    files.append(os.path.join(dp, f))
        # mod_system_* грузятся по алфавиту после остальных: позже - важнее
        files.sort(key=lambda p: (os.path.basename(p).lower().startswith('mod_system_'), os.path.basename(p).lower()))
        secs = {}  # section -> dict
        for p in files:
            cur = None
            for line in open(p, 'rb').read().decode('cp1251', 'replace').splitlines():
                line = line.split(';')[0].strip()
                m = re.match(r'^!?!?\[([^\]]+)\]', line)
                if m:
                    cur = secs.setdefault(m.group(1).strip().lower(), {})
                    continue
                if cur is None or '=' not in line:
                    continue
                k, v = line.split('=', 1)
                cur[k.strip().lower()] = v.strip()
        def vec(s):
            try:
                t = [float(x) for x in s.split(',')]
                return np.array(t[:3]) if len(t) >= 3 else None
            except ValueError:
                return None
        for s, kv in secs.items():
            vis = kv.get('item_visual', '').replace('/', '\\').lower()
            if not vis.startswith('attachments\\scopes\\'):
                continue
            name = vis.split('\\')[-1].replace('.ogf', '')
            pos = vec(kv.get('aim_hud_offset_pos', '')) if 'aim_hud_offset_pos' in kv else None
            alt = vec(kv.get('aim_hud_offset_alt_pos', '')) if 'aim_hud_offset_alt_pos' in kv else None
            if pos is not None and (name not in self.aim or s == name + '_hud'):
                self.aim[name] = (pos, alt)

    def find(self, model):
        """Эталон по текстуре корпуса; при нескольких - ближайший по числу вершин корпуса."""
        b = model.body()
        key, n = tex_key(b['tex']), len(b['P'])
        cands = [(abs(len(r['body_P']) - n), name) for name, r in self.models.items() if r['body_tex'] == key]
        if not cands:
            return None
        cands.sort()
        return cands[0][1]

# ---------------------------------------------------------------- конвертация
DEFAULTS = dict(default_scale=0.8974, clamp_bottom=-0.005, mount_band=0.010, aim_z=0.095)

def clamp_center(P, band):
    """Середина между зажимами: по слоям низа крепления ищется промежуток между губками (канал планки) 9..30 мм,
    центры голосуют. Возвращает x или None."""
    y0 = P[:, 1].min()
    votes = []
    step = 0.0015
    y = y0
    while y < y0 + band:
        xs = np.sort(P[(P[:, 1] >= y) & (P[:, 1] < y + step)][:, 0])
        y += step
        if len(xs) < 4:
            continue
        mid = (xs[0] + xs[-1]) / 2
        g = np.diff(xs)
        for i in np.where((g > 0.009) & (g < 0.030))[0]:
            c = (xs[i] + xs[i + 1]) / 2
            if xs[0] < xs[i] and xs[i + 1] < xs[-1] and abs(c - mid) < 0.02:  # губки с обеих сторон
                votes.append((c, g[i]))
    if not votes:
        return None
    cs = np.array([v[0] for v in votes])
    best, cnt = None, -1
    for c in cs:
        k = np.sum(np.abs(cs - c) < 0.0015)
        if k > cnt:
            best, cnt = c, k
    return float(np.median(cs[np.abs(cs - best) < 0.0015]))

def convert(path, refs, cfg, override=None):
    """Возвращает dict: out_bytes, report, points (для предпросмотра) и т.п. override: scale, dx, dy, dz (м)."""
    ov = override or {}
    m = Model(path)
    rep = dict(file=os.path.basename(path), notes=[])
    ref_name = refs.find(m) if refs else None
    ref = refs.models.get(ref_name) if ref_name else None
    # 1. масштаб
    b = m.body()
    if ov.get('scale'):
        s, how = float(ov['scale']), 'вручную'
    elif ref is not None and len(ref['body_P']) == len(b['P']):
        a, c = b['P'], ref['body_P']
        am, cm = a.mean(0), c.mean(0)
        s, how = float(((a - am) * (c - cm)).sum() / ((a - am) ** 2).sum()), 'как ' + ref_name + ' (тот же меш)'
    elif ref is not None and ref['ret_size'] is not None and m.reticle()[1] is not None and m.reticle()[1].min() > 1e-5:
        s, how = float(np.mean(ref['ret_size'] / m.reticle()[1])), 'как ' + ref_name + ' (по сетке)'
    elif ref is not None:
        s, how = float(np.mean(np.ptp(ref['body_P'], 0) / np.ptp(b['P'], 0))), 'как ' + ref_name + ' (по габаритам)'
    else:
        s, how = cfg['default_scale'], 'по умолчанию (эталона в Pizza нет)'
    rep['scale'], rep['scale_how'], rep['ref'] = s, how, ref_name or '-'
    P = m.all_points() * s
    # 2-4. положение по креплению
    band = cfg['mount_band']
    cx = clamp_center(P, band)
    ret, _ = m.reticle()
    if cx is None:
        cx = ret[0] * s if ret is not None else (P[:, 0].min() + P[:, 0].max()) / 2
        rep['notes'].append('зажимы не найдены: по центру ось сетки')
    y0 = P[:, 1].min()
    low = P[P[:, 1] < y0 + band]
    t = np.array([-cx, cfg['clamp_bottom'] - y0, -low[:, 2].min()])
    t += np.array([ov.get('dx', 0.0), ov.get('dy', 0.0), ov.get('dz', 0.0)])
    rep['t'] = t
    tf = lambda Q: (Q * s + t)
    # кости
    names = [b_[0] for b_ in m.bones]
    used = sorted({int(i) for k in m.kids for i in np.unique(k['B'])})
    lens = next((i for i, n in enumerate(names) if n.lower() == 'lens'), None)
    keep_lens = lens is not None and lens in used
    remap = lambda i: 1 if (keep_lens and i == lens) else 0
    # меши
    new_kids, allN = [], []
    for i, k in enumerate(m.kids):
        raw = dict(k['raw'])
        Pn = tf(k['P'])
        allN.append(Pn)
        sh = k['sh']
        if sh in SHADER_FIX:
            rep['notes'].append('%s -> %s' % (sh, SHADER_FIX[sh]))
            sh = SHADER_FIX[sh]
        raw[2] = k['tex'].encode('cp1251') + b'\0' + sh.encode('cp1251') + b'\0'
        Bn = np.vectorize(remap)(k['B']) if k['B'].size else k['B']
        raw[3] = write_verts(raw[3], Pn, Bn)
        raw[1] = header(raw[1], Pn)
        new_kids.append((i, pack(sorted(raw.items()))))
    allN = np.vstack(allN)
    keep = [0] + ([lens] if keep_lens else [])
    out_names = struct.pack('<I', len(keep))
    for n, o in enumerate(keep):
        obb = list(m.bones[o][2]) if o < len(m.bones) else [0.0] * 15
        obb[9:12] = list(np.array(obb[9:12]) * s + t)
        obb[12:15] = list(np.array(obb[12:15]) * s)
        out_names += (b'wpn_scope' if n == 0 else b'lens') + b'\0' + (b'' if n == 0 else b'wpn_scope') + b'\0' + struct.pack('<15f', *obb)
    out_ik = b''
    if 16 in m.T:
        ik = read_ik(m.T[16], len(m.bones))
        for o in keep:
            e = ik[o]
            p = e.index(b'\0', 4) + 1
            sb = p + 4
            for off, scale_only in ((sb + 36, False), (sb + 48, True), (sb + 60, False), (sb + 76, False)):
                v = np.array(struct.unpack_from('<3f', e, off))
                struct.pack_into('<3f', e, off, *(v * s if scale_only else v * s + t))
            for off in (sb + 72, sb + 100, sb + 104):
                struct.pack_into('<f', e, off, struct.unpack_from('<f', e, off)[0] * s)
            q = sb + 108 + 4 + 48 + 24
            struct.pack_into('<6f', e, q, 0, 0, 0, 0, 0, 0)
            out_ik += bytes(e)
    motion = 'attachments\\motions\\common_lens' if keep_lens else 'attachments\\motions\\common'
    refs_chunk = struct.pack('<I', 1) + motion.encode() + b'\0'
    chs = [(1, header(m.T[1], allN))]
    if 18 in m.T:
        chs.append((18, m.T[18]))
    chs += [(9, pack(new_kids)), (13, out_names)]
    if out_ik:
        chs.append((16, out_ik))
    chs.append((24, refs_chunk))
    out = pack(chs)
    rep['bones'] = ['wpn_scope'] + (['lens'] if keep_lens else [])
    rep['motion'] = motion
    # aim
    retn = tf(ret) if ret is not None else None
    aim = None
    if retn is not None:
        if ref is not None and ref_name in refs.aim and ref['ret'] is not None:
            aim = refs.aim[ref_name][0] - (retn - ref['ret'])
            rep['aim_how'] = 'от ' + ref_name
        else:
            aim = np.array([-retn[0], -retn[1], cfg['aim_z']])
            rep['aim_how'] = 'по оси сетки, z примерный - проверь'
    else:
        rep['aim_how'] = 'нет сетки - aim не посчитан'
    rep['aim'] = aim
    # проверки
    low = allN[allN[:, 1] < allN[:, 1].min() + band]
    rep['mount_rear'] = float(low[:, 2].min())
    rep['size'] = np.ptp(allN, 0)
    rep['clamp_x'] = float(clamp_center(allN, band) or 0.0)
    return dict(out=out, rep=rep, points=allN, kids=[(k['sh'], tf(k['P'])) for k in m.kids], band=band)

def fmt_aim(a):
    return '%.6f, %.6f, %.6f' % tuple(a)

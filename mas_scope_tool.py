"""MAS Scope Tool: пакетное приведение моделей прицелов (.ogf) к правилам MAS. Окно на tkinter."""
import os, sys, json, traceback
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import numpy as np
import mas_scope_core as C

APP = 'MAS Scope Tool'
HERE = os.path.dirname(sys.executable if getattr(sys, 'frozen', False) else os.path.abspath(__file__))
CFG_PATH = os.path.join(HERE, 'mas_scope_tool.json')
CFG_DEF = dict(C.DEFAULTS, pizza=r"C:\Games\GRA2.5\MO2\mods\[Прицелы 3д] Pizza's Ultimate Sight 18.02",
               out=r"C:\Games\GRA2.5\Development\result\MAS_Scopes\gamedata\meshes\attachments\scopes")

def load_cfg():
    c = dict(CFG_DEF)
    try:
        c.update(json.load(open(CFG_PATH, encoding='utf-8')))
    except Exception:
        pass
    return c

def save_cfg(c):
    try:
        json.dump(c, open(CFG_PATH, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    except Exception:
        pass

COL = {'reticle': '#e0453a', 'lens': '#3aa0e0', 'body': '#8a8a8a', 'mount': '#3cc85a'}

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title(APP)
        self.geometry('1280x820')
        self.cfg = load_cfg()
        self.refs = None
        self.items = []   # dict(path, ov, res, err)
        self._ui()
        self.after(100, self.load_refs)

    # ---------------- интерфейс
    def _ui(self):
        top = ttk.Frame(self, padding=6)
        top.pack(fill='x')
        self.v = {}
        def row(r, label, key, browse=None, w=70):
            ttk.Label(top, text=label).grid(row=r, column=0, sticky='w')
            var = tk.StringVar(value=str(self.cfg[key]))
            self.v[key] = var
            ttk.Entry(top, textvariable=var, width=w).grid(row=r, column=1, sticky='we', padx=4)
            if browse:
                ttk.Button(top, text='...', width=3, command=browse).grid(row=r, column=2)
        row(0, 'Мод Pizza (эталоны):', 'pizza', lambda: self._dir('pizza', self.load_refs))
        row(1, 'Папка вывода:', 'out', lambda: self._dir('out'))
        num = ttk.Frame(top)
        num.grid(row=2, column=0, columnspan=3, sticky='w', pady=4)
        for i, (k, t) in enumerate((('default_scale', 'Масштаб без эталона'), ('clamp_bottom', 'Низ крепления y, м'),
                                    ('mount_band', 'Высота крепления, м'), ('aim_z', 'aim z без эталона'))):
            ttk.Label(num, text=t).grid(row=0, column=2 * i, padx=(10 if i else 0, 2))
            var = tk.StringVar(value=str(self.cfg[k]))
            self.v[k] = var
            ttk.Entry(num, textvariable=var, width=9).grid(row=0, column=2 * i + 1)
        top.columnconfigure(1, weight=1)
        self.status = tk.StringVar(value='')
        ttk.Label(top, textvariable=self.status, foreground='#555').grid(row=3, column=0, columnspan=3, sticky='w')

        bar = ttk.Frame(self, padding=(6, 0))
        bar.pack(fill='x')
        for t, c in (('Добавить .ogf', self.add), ('Убрать', self.remove), ('Очистить', self.clear),
                     ('Обработать все', self.process_all), ('Сохранить все', self.save_all),
                     ('Копировать aim', self.copy_aim)):
            ttk.Button(bar, text=t, command=c).pack(side='left', padx=2, pady=4)

        main = ttk.PanedWindow(self, orient='horizontal')
        main.pack(fill='both', expand=True, padx=6, pady=6)
        left = ttk.Frame(main)
        main.add(left, weight=1)
        cols = ('ref', 'scale', 'aim', 'state')
        self.tree = ttk.Treeview(left, columns=cols, show='tree headings', selectmode='browse')
        self.tree.heading('#0', text='Файл')
        for c, t, w in (('ref', 'Эталон Pizza', 90), ('scale', 'Масштаб', 60), ('aim', 'aim_hud_offset_pos', 230), ('state', 'Состояние', 140)):
            self.tree.heading(c, text=t)
            self.tree.column(c, width=w, anchor='w')
        self.tree.column('#0', width=130)
        self.tree.pack(fill='both', expand=True)
        self.tree.bind('<<TreeviewSelect>>', lambda e: self.show())
        ov = ttk.LabelFrame(left, text='Поправки выбранного (мм, пусто = авто)', padding=4)
        ov.pack(fill='x', pady=4)
        self.ov = {}
        for i, (k, t) in enumerate((('scale', 'масштаб'), ('dx', 'dx'), ('dy', 'dy'), ('dz', 'dz'))):
            ttk.Label(ov, text=t).grid(row=0, column=2 * i, padx=(8 if i else 0, 2))
            var = tk.StringVar()
            self.ov[k] = var
            ttk.Entry(ov, textvariable=var, width=8).grid(row=0, column=2 * i + 1)
        ttk.Button(ov, text='Пересчитать', command=self.reprocess).grid(row=0, column=8, padx=8)
        self.info = tk.Text(left, height=12, wrap='word')
        self.info.pack(fill='both', expand=False)

        right = ttk.Frame(main)
        main.add(right, weight=2)
        self.canvas = tk.Canvas(right, background='#1e1f22', highlightthickness=0)
        self.canvas.pack(fill='both', expand=True)
        self.canvas.bind('<Configure>', lambda e: self.draw())
        ttk.Label(right, text='Зелёное - крепление; зелёная линия - ось планки (x = 0); жёлтые - y = 0 и z = 0; красное - сетка.',
                  foreground='#555').pack(anchor='w')

    def _dir(self, key, after=None):
        d = filedialog.askdirectory(initialdir=self.v[key].get() or HERE)
        if d:
            self.v[key].set(os.path.normpath(d))
            if after:
                after()

    def read_cfg(self):
        c = dict(self.cfg)
        for k, var in self.v.items():
            c[k] = var.get().strip()
            if k in C.DEFAULTS:
                try:
                    c[k] = float(c[k].replace(',', '.'))
                except ValueError:
                    c[k] = C.DEFAULTS[k]
        self.cfg = c
        save_cfg(c)
        return c

    def load_refs(self):
        c = self.read_cfg()
        self.status.set('Читаю эталоны Pizza...')
        self.update_idletasks()
        try:
            self.refs = C.PizzaRefs(c['pizza'])
            self.status.set(self.refs.error or 'Эталоны Pizza: моделей %d, aim из конфигов %d' % (len(self.refs.models), len(self.refs.aim)))
        except Exception as e:
            self.refs = None
            self.status.set('Эталоны Pizza не прочитаны: %s' % e)

    # ---------------- список
    def add(self):
        fs = filedialog.askopenfilenames(filetypes=[('OGF', '*.ogf'), ('Все', '*.*')])
        for f in fs:
            if any(it['path'] == f for it in self.items):
                continue
            it = dict(path=f, ov={}, res=None, err=None)
            self.items.append(it)
            it['id'] = self.tree.insert('', 'end', text=os.path.basename(f), values=('', '', '', 'не обработан'))
        if fs:
            self.process_all()

    def sel(self):
        s = self.tree.selection()
        return next((it for it in self.items if it['id'] == (s[0] if s else None)), None)

    def remove(self):
        it = self.sel()
        if it:
            self.tree.delete(it['id'])
            self.items.remove(it)
            self.draw()

    def clear(self):
        for it in self.items:
            self.tree.delete(it['id'])
        self.items = []
        self.draw()

    def process(self, it):
        c = self.read_cfg()
        try:
            it['res'] = C.convert(it['path'], self.refs, c, it['ov'])
            it['err'] = None
            r = it['res']['rep']
            aim = C.fmt_aim(r['aim']) if r['aim'] is not None else '-'
            state = 'готов' if 'эталон' not in r['scale_how'] and 'примерный' not in r.get('aim_how', '') else 'проверь'
            self.tree.item(it['id'], values=(r['ref'], '%.4f' % r['scale'], aim, state))
        except Exception as e:
            it['res'], it['err'] = None, traceback.format_exc()
            self.tree.item(it['id'], values=('', '', '', 'ошибка: %s' % e))

    def process_all(self):
        for it in self.items:
            self.process(it)
        if self.items and not self.tree.selection():
            self.tree.selection_set(self.items[0]['id'])
        self.show()

    def reprocess(self):
        it = self.sel()
        if not it:
            return
        ov = {}
        for k, var in self.ov.items():
            t = var.get().strip().replace(',', '.')
            if t:
                try:
                    ov[k] = float(t) if k == 'scale' else float(t) / 1000.0
                except ValueError:
                    pass
        it['ov'] = ov
        self.process(it)
        self.show()

    def save_all(self):
        c = self.read_cfg()
        out = c['out']
        ok = [it for it in self.items if it['res']]
        if not ok:
            return
        try:
            os.makedirs(out, exist_ok=True)
            lines = []
            for it in ok:
                r = it['res']['rep']
                open(os.path.join(out, os.path.basename(it['path'])), 'wb').write(it['res']['out'])
                name = os.path.splitext(os.path.basename(it['path']))[0]
                lines.append('; %s  (эталон: %s, масштаб %.4f; aim: %s)' % (name, r['ref'], r['scale'], r.get('aim_how', '')))
                if r['aim'] is not None:
                    a = C.fmt_aim(r['aim'])
                    lines += ['aim_hud_offset_pos      = ' + a, 'aim_hud_offset_pos_16x9 = ' + a]
                lines.append('')
            open(os.path.join(out, 'mas_scopes_aim.txt'), 'w', encoding='cp1251', newline='\r\n').write('\n'.join(lines))
            self.status.set('Сохранено %d моделей и mas_scopes_aim.txt в %s' % (len(ok), out))
        except Exception as e:
            messagebox.showerror(APP, 'Не удалось сохранить: %s' % e)

    def copy_aim(self):
        it = self.sel()
        if not (it and it['res'] and it['res']['rep']['aim'] is not None):
            return
        a = C.fmt_aim(it['res']['rep']['aim'])
        self.clipboard_clear()
        self.clipboard_append('aim_hud_offset_pos      = %s\naim_hud_offset_pos_16x9 = %s\n' % (a, a))
        self.status.set('aim скопирован')

    # ---------------- показ
    def show(self):
        it = self.sel()
        self.info.delete('1.0', 'end')
        if it:
            for k, var in self.ov.items():
                v = it['ov'].get(k)
                var.set('' if v is None else ('%g' % v if k == 'scale' else '%g' % (v * 1000)))
            if it['err']:
                self.info.insert('end', it['err'])
            elif it['res']:
                r = it['res']['rep']
                t = []
                t.append('Эталон Pizza: %s' % r['ref'])
                t.append('Масштаб: %.4f - %s' % (r['scale'], r['scale_how']))
                t.append('Размер: %.1f x %.1f x %.1f мм' % tuple(r['size'] * 1000))
                t.append('Сдвиг: %.4f, %.4f, %.4f' % tuple(r['t']))
                t.append('Кости: %s; анимация: %s' % (', '.join(r['bones']), r['motion']))
                t.append('Центр крепления x = %.1f мм, задний край крепления z = %.1f мм' % (r['clamp_x'] * 1000, r['mount_rear'] * 1000))
                t.append('aim (%s):' % r.get('aim_how', ''))
                if r['aim'] is not None:
                    t.append('  aim_hud_offset_pos      = ' + C.fmt_aim(r['aim']))
                    t.append('  aim_hud_offset_pos_16x9 = ' + C.fmt_aim(r['aim']))
                t += ['- ' + n for n in r['notes']]
                self.info.insert('end', '\n'.join(t))
        self.draw()

    def draw(self):
        cv = self.canvas
        cv.delete('all')
        it = self.sel()
        if not (it and it['res']):
            return
        W, H = cv.winfo_width(), cv.winfo_height()
        res = it['res']
        P = res['points']
        lo, hi = P.min(0), P.max(0)
        band = P[:, 1].min() + res['band']
        views = (('Сбоку (z вперёд)', 2, 1, False), ('Спереди (x вправо)', 0, 1, False), ('Сверху', 2, 0, True))
        pad = 30
        cell_w, cell_h = (W - pad * 3) / 2, (H - pad * 3) / 2
        boxes = ((pad, pad), (pad * 2 + cell_w, pad), (pad, pad * 2 + cell_h))
        rng = np.ptp(P, 0).max() * 1.25 + 1e-6
        for (title, a, b, flip), (bx, by) in zip(views, boxes):
            k = min(cell_w, cell_h) / rng
            ca, cb = (lo[a] + hi[a]) / 2, (lo[b] + hi[b]) / 2
            ox, oy = bx + cell_w / 2, by + cell_h / 2
            X = lambda u: ox + (u - ca) * k
            Y = lambda v: oy - (v - cb) * k if not flip else oy + (v - cb) * k
            cv.create_text(bx, by - 14, text=title, anchor='w', fill='#ccc')
            # оси
            if a == 2:
                cv.create_line(X(0), by, X(0), by + cell_h, fill='#d0b020', dash=(3, 3))
            if b == 1:
                cv.create_line(bx, Y(0), bx + cell_w, Y(0), fill='#d0b020', dash=(3, 3))
            if a == 0:
                cv.create_line(X(0), by, X(0), by + cell_h, fill=COL['mount'], width=2)
            if b == 0:
                cv.create_line(bx, Y(0), bx + cell_w, Y(0), fill=COL['mount'], width=2)
            for sh, Q in res['kids']:
                col = COL['reticle'] if 'reticle' in sh else COL['lens'] if 'lens' in sh else None
                step = max(1, len(Q) // 2500)
                for p in Q[::step]:
                    c = col or (COL['mount'] if p[1] < band else COL['body'])
                    x, y = X(p[a]), Y(p[b])
                    cv.create_rectangle(x, y, x + 1, y + 1, outline=c, fill=c)

def main():
    App().mainloop()

if __name__ == '__main__':
    main()

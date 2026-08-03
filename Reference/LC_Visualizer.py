import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
from matplotlib.gridspec import GridSpec
from scipy import stats
from scipy.interpolate import make_interp_spline

_CN_FONTS = ['Microsoft YaHei', 'SimHei', 'WenQuanYi Micro Hei', 'Noto Sans CJK SC', 'Arial Unicode MS']
_cn = None
_available = {f.name for f in matplotlib.font_manager.fontManager.ttflist}
for _f in _CN_FONTS:
    if _f in _available:
        _cn = _f
        break

plt.rcParams.update({
    'font.size': 10, 'axes.titlesize': 13, 'axes.labelsize': 11,
    'xtick.labelsize': 9, 'ytick.labelsize': 9, 'legend.fontsize': 8,
    'figure.dpi': 120, 'savefig.dpi': 200, 'savefig.bbox': 'tight', 'savefig.pad_inches': 0.2,
})
if _cn:
    plt.rcParams['font.sans-serif'] = [_cn] + plt.rcParams.get('font.sans-serif', [])
    plt.rcParams['axes.unicode_minus'] = False

_PAL = ['#2C5F8A','#1A8A7D','#D4742B','#C0392B','#6C3483','#B53471',
        '#B7950B','#3498db','#2ecc71','#e74c3c','#f39c12','#9b59b6',
        '#1abc9c','#e67e22','#34495e','#27ae60','#d35400','#7f8c8d']

# ============================================================
_VM = {
    'I_rms':       {'label': 'RMS回路电流 I_rms',        'unit': 'mA',   'scale': 1e3},
    'I_peak':      {'label': '峰值电流 I_peak',           'unit': 'mA',   'scale': 1e3},
    'V_out_pp':    {'label': '驱动电压 V_out_pp',          'unit': 'Vpp',  'scale': 1},
    'Q':           {'label': '品质因数 Q',                 'unit': '',     'scale': 1},
    'Z_total':     {'label': '回路阻抗 |Z|',              'unit': 'Ω',    'scale': 1},
    'f_res':       {'label': '谐振频率 f_res',            'unit': 'kHz',  'scale': 1e-3},
    'P_ch_total':  {'label': '单通道功耗 P_ch_total',     'unit': 'W',    'scale': 1},
    'P_board_total':{'label': '整板功耗 P_board_total',   'unit': 'W',    'scale': 1},
    'P_Rd':        {'label': '阻尼电阻发热 P_Rd',         'unit': 'W',    'scale': 1},
    'P_L':         {'label': '电感发热 P_L',              'unit': 'W',    'scale': 1},
    'P_IC':        {'label': '驱动IC发热 P_IC',           'unit': 'W',    'scale': 1},
    'P_rad':       {'label': '辐射声功率 P_rad',           'unit': 'W',    'scale': 1},
    'L_actual':    {'label': '有效电感 L_actual',         'unit': 'mH',   'scale': 1e3},
    'X_C':         {'label': '容抗 X_C',                  'unit': 'Ω',    'scale': 1},
    'X_L':         {'label': '感抗 X_L',                  'unit': 'Ω',    'scale': 1},
    'R_total':     {'label': '回路总电阻 R_total',        'unit': 'Ω',    'scale': 1},
}

class UltrasonicResonanceVisualizer:
    def __init__(self, calculator):
        self.calc = calculator
        self._cache = None

    @property
    def _data(self):
        if self._cache is None:
            self._cache = self.calc._enumerate_results()
        return self._cache

    def _arr(self, metric):
        scale = _VM[metric]['scale']
        return np.array([r[metric] * scale for r in self._data[2]])

    def _input_arrs(self):
        keys = self._data[0]
        inp = np.array(self._data[3])
        return {k: inp[:, i] * self.calc.components[k]['scale']
                for i, k in enumerate(keys)}

    # ------------------------------------------------------------------
    # 1. KDE 概率密度分布
    # ------------------------------------------------------------------
    def plot_kde_distributions(self, save_path=None, figsize=(18, 22)):
        groups = [
            ('电流特性',     ['I_rms', 'I_peak']),
            ('电压与谐振',   ['V_out_pp', 'Q', 'f_res']),
            ('阻抗特性',     ['Z_total', 'L_actual', 'X_C', 'X_L']),
            ('功耗分布',     ['P_ch_total', 'P_board_total', 'P_Rd', 'P_L', 'P_IC', 'P_rad']),
        ]
        n_cols = 2
        rows_per_page = 3

        all_subs = []
        for _, gms in groups:
            all_subs.extend(gms)
        per_page = n_cols * rows_per_page
        n_pages = (len(all_subs) + per_page - 1) // per_page

        for pi in range(n_pages):
            fig, axes = plt.subplots(rows_per_page, n_cols, figsize=figsize)
            axes = axes.flatten()
            fig.suptitle(f'超声相控阵 LC 谐振驱动 — KDE 概率密度    第{pi+1}/{n_pages}页',
                         fontsize=14, fontweight='bold', y=0.98)

            start = pi * per_page
            end = min(start + per_page, len(all_subs))
            for ai in range(per_page):
                ax = axes[ai]
                idx = start + ai
                if idx >= len(all_subs):
                    ax.set_visible(False)
                    continue

                metric = all_subs[idx]
                data = self._arr(metric)
                info = _VM[metric]
                dc = data[np.isfinite(data)]

                kde = stats.gaussian_kde(dc)
                xk = np.linspace(dc.min() * 0.95, dc.max() * 1.05, 300)
                yk = kde(xk)

                ax.plot(xk, yk, color=_PAL[ai % len(_PAL)], linewidth=2, label='KDE')
                ax.hist(dc, bins=40, density=True, alpha=0.3,
                        color=_PAL[ai % len(_PAL)], edgecolor='white', linewidth=0.3)
                ax.axvline(np.median(dc), color='black', linestyle='--', linewidth=1,
                           label=f'Median: {np.median(dc):.4g}')
                ax.axvline(np.percentile(dc, 5), color='red', linestyle=':', linewidth=0.8)
                ax.axvline(np.percentile(dc, 95), color='red', linestyle=':', linewidth=0.8,
                           label=f'P5-P95')

                u = f' ({info["unit"]})' if info['unit'] else ''
                ax.set_title(f'{info["label"]}{u}', fontsize=11, fontweight='bold')
                ax.set_ylabel('概率密度', fontsize=9)
                ax.set_xlabel(info['unit'], fontsize=9)
                ax.legend(fontsize=7, loc='upper right')
                ax.grid(axis='y', alpha=0.2)

            for ax in axes[end - start:]:
                ax.set_visible(False)

            fig.tight_layout(rect=[0, 0, 1, 0.95])
            if save_path:
                b, e = os.path.splitext(save_path)
                p = f'{b}_kde_p{pi+1}{e or ".png"}'
                fig.savefig(p)
                print(f'  [KDE] -> {p}')
            plt.close(fig)

    # ------------------------------------------------------------------
    # 2. 灵敏度龙卷风图
    # ------------------------------------------------------------------
    def plot_sensitivity_tornado(self, target_metrics=None, save_path=None, figsize=(20, 26)):
        if target_metrics is None:
            target_metrics = ['I_rms','I_peak','V_out_pp','Q','Z_total',
                              'P_ch_total','P_board_total','P_Rd','P_L','P_IC']
        keys = self._data[0]
        nom_vals = {k: self.calc.components[k]['nom'] for k in keys}
        nom_res = self.calc._calc_state(nom_vals)

        pct = {}
        for p in keys:
            nom = self.calc.components[p]['nom']
            tl = self.calc.components[p]['tol']

            vn = nom * (1.0 - tl) if tl > 1e-9 else nom
            vv = dict(nom_vals); vv[p] = vn
            rn = self.calc._calc_state(vv)

            vp = nom * (1.0 + tl) if tl > 1e-9 else nom
            vv = dict(nom_vals); vv[p] = vp
            rp = self.calc._calc_state(vv)

            for m in target_metrics:
                nv = nom_res[m]
                dn = (rn[m] - nv) / nv * 100 if abs(nv) > 1e-15 else 0.0
                dp = (rp[m] - nv) / nv * 100 if abs(nv) > 1e-15 else 0.0
                pct.setdefault(m, {})[p] = (dn, dp)

        n_cols = 2
        n_rows = (len(target_metrics) + n_cols - 1) // n_cols
        fig, axes = plt.subplots(n_rows, n_cols, figsize=figsize)
        axes = np.atleast_1d(axes).flatten()
        fig.suptitle('元件灵敏度分析 (龙卷风图)', fontsize=15, fontweight='bold', y=1.01)

        for i, m in enumerate(target_metrics):
            ax = axes[i]
            ch = pct[m]
            pns = list(ch.keys())
            lbs = [self.calc.components[p]['desc'] + f'\n({p})' for p in pns]
            nv = [ch[p][0] for p in pns]
            pv = [ch[p][1] for p in pns]
            yp = np.arange(len(pns))
            bh = 0.35

            ax.barh(yp, nv, bh, color='#3498db', label='-tol%', edgecolor='white')
            ax.barh(yp, pv, bh, color='#e74c3c', label='+tol%', edgecolor='white')
            ax.set_yticks(yp)
            ax.set_yticklabels(lbs, fontsize=8)
            ax.set_xlabel('变化率 (%)', fontsize=9)
            ax.set_title(f'{_VM[m]["label"]} 灵敏度', fontsize=11, fontweight='bold')
            ax.axvline(0, color='black', linewidth=0.8)
            ax.legend(fontsize=7)
            ax.grid(axis='x', alpha=0.2)

        for j in range(i + 1, len(axes)):
            axes[j].set_visible(False)

        fig.tight_layout(rect=[0, 0, 1, 0.96])
        if save_path:
            b, e = os.path.splitext(save_path)
            fig.savefig(f'{b}_sensitivity{e or ".png"}')
            print(f'  [灵敏度] -> {b}_sensitivity{e or ".png"}')
        plt.close(fig)
        return fig

    # ------------------------------------------------------------------
    # 3. 关联热力图
    # ------------------------------------------------------------------
    def plot_correlation_heatmap(self, save_path=None, figsize=(16, 10)):
        inp = self._input_arrs()
        ik = list(inp.keys())
        idata = np.column_stack([inp[k] for k in ik])

        om = ['I_rms','I_peak','V_out_pp','Q','Z_total','f_res',
              'P_Rd','P_L','P_IC','P_rad','P_ch_total','P_board_total','L_actual']
        odata = np.column_stack([self._arr(m) for m in om])

        ad = np.column_stack([idata, odata])
        corr = np.corrcoef(ad, rowvar=False)
        cs = corr[:len(ik), len(ik):]

        fig, ax = plt.subplots(figsize=figsize)
        im = ax.imshow(cs, cmap='RdBu_r', aspect='auto', vmin=-1, vmax=1)
        ax.set_xticks(range(len(om)))
        ax.set_xticklabels([_VM[m]['label'] for m in om], rotation=45, ha='right', fontsize=8)
        ax.set_yticks(range(len(ik)))
        ax.set_yticklabels([self.calc.components[k]['desc'] for k in ik], fontsize=9)

        for i in range(len(ik)):
            for j in range(len(om)):
                v = cs[i, j]
                c = 'white' if abs(v) > 0.6 else 'black'
                ax.text(j, i, f'{v:.3f}', ha='center', va='center', fontsize=8,
                        color=c, fontweight='bold')

        cbar = fig.colorbar(im, ax=ax, shrink=0.85)
        cbar.set_label('Pearson r', fontsize=10)
        ax.set_title('输入参数 vs 输出指标 皮尔逊相关系数矩阵', fontsize=13, fontweight='bold')
        fig.tight_layout()

        if save_path:
            b, e = os.path.splitext(save_path)
            fig.savefig(f'{b}_correlation{e or ".png"}')
            print(f'  [热力图] -> {b}_correlation{e or ".png"}')
        plt.close(fig)
        return fig

    # ------------------------------------------------------------------
    # 4. 单参数连续扫描
    # ------------------------------------------------------------------
    def plot_parameter_sweep(self, num_points=100, save_path=None, figsize=(22, 28)):
        keys = self._data[0]
        nd = {k: self.calc.components[k]['nom'] for k in keys}
        watched = ['I_peak','V_out_pp','Q','f_res','I_rms','P_ch_total','P_board_total']

        n_cols = 2
        n_rows = (len(keys) + n_cols - 1) // n_cols
        fig, axes = plt.subplots(n_rows, n_cols, figsize=figsize)
        axes = np.atleast_1d(axes).flatten()
        fig.suptitle('单参数连续扫描 — 公差范围内输出变化趋势', fontsize=15, fontweight='bold')

        for idx, param in enumerate(keys):
            ax = axes[idx]
            nom = self.calc.components[param]['nom']
            tl = self.calc.components[param]['tol']
            sc = self.calc.components[param]['scale']

            if tl < 1e-9:
                sw = np.array([nom])
            else:
                sw = np.linspace(nom * (1 - tl), nom * (1 + tl), num_points)

            rl = []
            for v in sw:
                tv = dict(nd); tv[param] = v
                rl.append(self.calc._calc_state(tv))

            for j, m in enumerate(watched):
                info = _VM[m]
                yv = np.array([r[m] * info['scale'] for r in rl])
                xv = sw * sc
                ax.plot(xv, yv, linewidth=1.5, color=_PAL[j % len(_PAL)],
                        label=f'{info["label"]} ({info["unit"]})', alpha=0.8)

            ax.axvline(nom * sc, color='gray', linestyle=':', linewidth=0.8, alpha=0.5)
            ax.set_xlabel(f'{self.calc.components[param]["desc"]} '
                          f'({self.calc.components[param]["unit"]})', fontsize=9)
            ttl_u = self.calc.components[param]['unit']
            ax.set_title(f'{self.calc.components[param]["desc"]} ({param}) '
                         f'[{nom*(1-tl)*sc:.3g}~{nom*(1+tl)*sc:.3g} {ttl_u}]',
                         fontsize=9, fontweight='bold')
            ax.legend(fontsize=6, loc='best', ncol=2)
            ax.grid(alpha=0.2)

        for j in range(idx + 1, len(axes)):
            axes[j].set_visible(False)

        fig.tight_layout(rect=[0, 0, 1, 0.96])
        if save_path:
            b, e = os.path.splitext(save_path)
            fig.savefig(f'{b}_sweep{e or ".png"}')
            print(f'  [参数扫描] -> {b}_sweep{e or ".png"}')
        plt.close(fig)
        return fig

    # ------------------------------------------------------------------
    # 5. 散点矩阵图
    # ------------------------------------------------------------------
    def plot_scatter_matrix(self, save_path=None, figsize=(24, 18)):
        om = ['I_peak','V_out_pp','Q','P_board_total']
        ip = ['L','DCR','R_d','C_t']
        inp = self._input_arrs()
        od = {m: self._arr(m) for m in om}

        n_rows = len(om); n_cols = len(ip)
        fig, axes = plt.subplots(n_rows, n_cols, figsize=figsize,
                                  gridspec_kw={'hspace': 0.35, 'wspace': 0.3})
        fig.suptitle('关键输入 vs 输出 散点矩阵分布', fontsize=15, fontweight='bold')

        cdata = self._arr('I_peak')
        norm = plt.Normalize(vmin=cdata.min(), vmax=cdata.max())

        for ri, out_m in enumerate(om):
            for ci, in_p in enumerate(ip):
                ax = axes[ri][ci] if n_rows > 1 else axes[ci]
                x = inp[in_p]
                y = od[out_m]
                ax.scatter(x, y, c=cdata, cmap='plasma', norm=norm,
                           s=5, alpha=0.35, edgecolors='none')
                try:
                    coeff = np.polyfit(x, y, 1)
                    pf = np.poly1d(coeff)
                    xl = np.linspace(x.min(), x.max(), 100)
                    ax.plot(xl, pf(xl), color='black', linewidth=1, linestyle='--', alpha=0.6)
                except Exception:
                    pass
                ax.set_xlabel(f'{self.calc.components[in_p]["desc"]} '
                              f'({self.calc.components[in_p]["unit"]})', fontsize=8)
                ax.set_ylabel(f'{_VM[out_m]["label"]} ({_VM[out_m]["unit"]})', fontsize=8)
                ax.grid(alpha=0.2)

        cax = fig.add_axes([0.93, 0.08, 0.015, 0.84])
        sm = plt.cm.ScalarMappable(cmap='plasma', norm=norm)
        fig.colorbar(sm, cax=cax).set_label('I_peak (mA)', fontsize=9)

        if save_path:
            b, e = os.path.splitext(save_path)
            fig.savefig(f'{b}_scatter{e or ".png"}')
            print(f'  [散点矩阵] -> {b}_scatter{e or ".png"}')
        plt.close(fig)
        return fig

    # ------------------------------------------------------------------
    # 6. CDF 累积分布
    # ------------------------------------------------------------------
    def plot_cdf_curves(self, save_path=None, figsize=(20, 16)):
        core = ['I_peak','V_out_pp','Q','f_res','Z_total',
                'P_ch_total','P_board_total','I_rms']
        n_cols = 3
        n_rows = (len(core) + n_cols - 1) // n_cols
        fig, axes = plt.subplots(n_rows, n_cols, figsize=figsize)
        axes = np.atleast_1d(axes).flatten()
        fig.suptitle('核心指标累积分布函数 (CDF)', fontsize=14, fontweight='bold')

        for i, m in enumerate(core):
            ax = axes[i]
            d = self._arr(m)
            info = _VM[m]
            ds = np.sort(d)
            n = len(ds)
            cdf = np.arange(1, n + 1) / n

            ax.step(ds, cdf, where='post', color=_PAL[i % len(_PAL)], linewidth=2, alpha=0.9)
            ax.fill_between(ds, cdf, alpha=0.15, color=_PAL[i % len(_PAL)], step='post')

            for pct in [5, 50, 95]:
                v = np.percentile(d, pct)
                cv = pct / 100.0
                ax.axhline(cv, color='gray', linestyle=':', linewidth=0.6, alpha=0.5)
                ax.axvline(v, color='gray', linestyle=':', linewidth=0.6, alpha=0.5)
                ax.plot(v, cv, 'o', markersize=5, color=_PAL[i % len(_PAL)])
                ax.text(v, cv + 0.05, f'P{pct}={v:.3g}', fontsize=7, ha='center', fontweight='bold')

            u = f' ({info["unit"]})' if info['unit'] else ''
            ax.set_title(f'{info["label"]}{u}', fontsize=10, fontweight='bold')
            ax.set_ylabel('累积概率', fontsize=9)
            ax.set_xlabel(info['unit'], fontsize=9)
            ax.grid(alpha=0.2)

        for j in range(i + 1, len(axes)):
            axes[j].set_visible(False)

        fig.tight_layout(rect=[0, 0, 1, 0.95])
        if save_path:
            b, e = os.path.splitext(save_path)
            fig.savefig(f'{b}_cdf{e or ".png"}')
            print(f'  [CDF] -> {b}_cdf{e or ".png"}')
        plt.close(fig)
        return fig

    # ------------------------------------------------------------------
    # 7. 箱线图 + 小提琴图
    # ------------------------------------------------------------------
    def plot_boxplot(self, save_path=None, figsize=(20, 14)):
        core = ['I_peak','V_out_pp','Q','f_res','Z_total',
                'P_ch_total','P_board_total','I_rms']
        n_cols = 3
        n_rows = (len(core) + n_cols - 1) // n_cols
        fig, axes = plt.subplots(n_rows, n_cols, figsize=figsize)
        axes = np.atleast_1d(axes).flatten()
        fig.suptitle('箱线图 + 小提琴分布', fontsize=14, fontweight='bold')

        for i, m in enumerate(core):
            ax = axes[i]
            d = self._arr(m)
            info = _VM[m]

            vp = ax.violinplot(d, positions=[0], vert=True, showmeans=True,
                               showmedians=True, widths=0.6)
            for b in vp['bodies']:
                b.set_facecolor(_PAL[i % len(_PAL)])
                b.set_alpha(0.5)
            for part in ['cmeans','cmedians','cbars','cmins','cmaxes']:
                if part in vp:
                    vp[part].set_color(_PAL[i % len(_PAL)])

            bp = ax.boxplot(d, positions=[0], widths=0.3, patch_artist=True,
                            showfliers=True, showmeans=True,
                            meanprops=dict(marker='D', markerfacecolor='red', markersize=6))
            for patch in bp['boxes']:
                patch.set_facecolor('white')
                patch.set_alpha(0.7)

            ax.set_xticks([0])
            ax.set_xticklabels([info['label']], fontsize=8, rotation=20)
            u = f' ({info["unit"]})' if info['unit'] else ''
            ax.set_ylabel(info['unit'], fontsize=9)
            ax.set_title(f'{info["label"]}{u}', fontsize=10, fontweight='bold')
            ax.grid(axis='y', alpha=0.2)

        for j in range(i + 1, len(axes)):
            axes[j].set_visible(False)

        fig.tight_layout(rect=[0, 0, 1, 0.95])
        if save_path:
            b, e = os.path.splitext(save_path)
            fig.savefig(f'{b}_boxplot{e or ".png"}')
            print(f'  [箱线图] -> {b}_boxplot{e or ".png"}')
        plt.close(fig)
        return fig

    # ------------------------------------------------------------------
    # 8. 谐振匹配质量综合评估
    # ------------------------------------------------------------------
    def plot_resonance_quality(self, save_path=None, figsize=(22, 12)):
        keys = self._data[0]
        nd = {k: self.calc.components[k]['nom'] for k in keys}
        results = self._data[2]

        fig = plt.figure(figsize=figsize, constrained_layout=True)
        gs = GridSpec(2, 3, figure=fig)

        # f_res 分布
        ax1 = fig.add_subplot(gs[0, 0])
        fres = np.array([r['f_res'] for r in results]) * 1e-3
        ax1.hist(fres, bins=50, color='#3498db', edgecolor='white', alpha=0.7)
        ax1.axvline(self.calc.f * 1e-3, color='red', linewidth=2, linestyle='--',
                     label=f'Target: {self.calc.f*1e-3:.1f} kHz')
        ax1.axvspan((self.calc.f - 500) * 1e-3, (self.calc.f + 500) * 1e-3,
                     alpha=0.15, color='green', label='+/-500 Hz')
        ax1.set_xlabel('f_res (kHz)')
        ax1.set_ylabel('Count')
        ax1.set_title('物理谐振频率分布', fontsize=11, fontweight='bold')
        ax1.legend(fontsize=7)
        ax1.grid(alpha=0.2)

        # |Z| vs X_C
        ax2 = fig.add_subplot(gs[0, 1])
        z = np.array([r['Z_total'] for r in results])
        xc = np.array([r['X_C'] for r in results])
        ax2.scatter(z, xc, c='#e74c3c', s=8, alpha=0.3, edgecolors='none')
        ax2.plot([0, max(z)], [0, max(z)], 'k--', linewidth=0.8, alpha=0.5,
                 label='Z = X_C')
        ax2.set_xlabel('|Z| (ohm)')
        ax2.set_ylabel('X_C (ohm)')
        ax2.set_title('阻抗 vs 容抗', fontsize=11, fontweight='bold')
        ax2.legend(fontsize=7)
        ax2.grid(alpha=0.2)

        # Q 分布
        ax3 = fig.add_subplot(gs[0, 2])
        qd = np.array([r['Q'] for r in results])
        ax3.hist(qd, bins=50, color='#2ecc71', edgecolor='white', alpha=0.7)
        ax3.axvline(np.median(qd), color='black', linestyle='--', linewidth=1.5,
                     label=f'Median Q = {np.median(qd):.3f}')
        ax3.set_xlabel('Q')
        ax3.set_ylabel('Count')
        ax3.set_title('品质因数分布', fontsize=11, fontweight='bold')
        ax3.legend(fontsize=7)
        ax3.grid(alpha=0.2)

        # I_peak vs V_out_pp
        ax4 = fig.add_subplot(gs[1, 0])
        idata = self._arr('I_peak')
        vdata = self._arr('V_out_pp')
        sc = ax4.scatter(idata, vdata, c=self._arr('P_board_total'), cmap='hot',
                         s=6, alpha=0.4, edgecolors='none')
        ax4.set_xlabel('I_peak (mA)')
        ax4.set_ylabel('V_out_pp (Vpp)')
        ax4.set_title('峰值电流 vs 驱动电压 (着色=整板功耗)', fontsize=11, fontweight='bold')
        fig.colorbar(sc, ax=ax4, shrink=0.8).set_label('P_board_total (W)', fontsize=8)
        ax4.grid(alpha=0.2)

        # 功耗分解饼图
        ax5 = fig.add_subplot(gs[1, 1])
        nr = self.calc._calc_state(nd)
        pie_d = [nr['P_Rd'], nr['P_L'], nr['P_IC'], nr['P_rad'], nr['P_stat']]
        pie_l = [f'R_d\n{nr["P_Rd"]:.3f}W', f'DCR\n{nr["P_L"]:.3f}W',
                 f'IC\n{nr["P_IC"]:.3f}W', f'Radiation\n{nr["P_rad"]:.3f}W',
                 f'Static\n{nr["P_stat"]:.3f}W']
        cl = ['#e74c3c','#f39c12','#3498db','#2ecc71','#95a5a6']
        ax5.pie(pie_d, labels=pie_l, colors=cl, autopct='%1.1f%%', startangle=90,
                explode=(0.03, 0.03, 0.03, 0.03, 0.03))
        ax5.set_title(f'标称值单通道功耗分解 (总: {nr["P_ch_total"]:.3f}W)',
                      fontsize=11, fontweight='bold')

        # 整板功耗 CDF
        ax6 = fig.add_subplot(gs[1, 2])
        pd = self._arr('P_board_total')
        ps = np.sort(pd)
        n = len(ps)
        cdf_v = np.arange(1, n + 1) / n
        ax6.step(ps, cdf_v, where='post', color='#e74c3c', linewidth=2)
        ax6.fill_between(ps, cdf_v, alpha=0.15, color='#e74c3c', step='post')
        ax6.axvline(36.0, color='red', linewidth=2, linestyle='--', label='36W 整板限制')
        over = np.sum(pd > 36.0) / len(pd) * 100
        ax6.text(0.98, 0.12,
                 f'>36W: {over:.1f}%\n'
                 f'P5={np.percentile(pd,5):.1f}W P50={np.percentile(pd,50):.1f}W\n'
                 f'P95={np.percentile(pd,95):.1f}W',
                 transform=ax6.transAxes, fontsize=9, ha='right',
                 bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        ax6.set_xlabel('P_board_total (W)')
        ax6.set_ylabel('CDF')
        ax6.set_title('整板功耗 CDF + 36W限制', fontsize=11, fontweight='bold')
        ax6.legend(fontsize=7)
        ax6.grid(alpha=0.2)

        if save_path:
            b, e = os.path.splitext(save_path)
            fig.savefig(f'{b}_quality{e or ".png"}')
            print(f'  [谐振质量] -> {b}_quality{e or ".png"}')
        plt.close(fig)
        return fig

    # ------------------------------------------------------------------
    # 9. 双参数联合分布热力图
    # ------------------------------------------------------------------
    def plot_2d_joint_distributions(self, save_path=None, figsize=(22, 22)):
        pairs = [
            ('L','DCR','I_peak','电感 vs DCR -> 峰值电流'),
            ('L','C_t','V_out_pp','电感 vs 电容 -> 驱动电压'),
            ('R_d','DCR','P_board_total','阻尼电阻 vs DCR -> 整板功耗'),
            ('L','R_rad','Q','电感 vs 辐射阻抗 -> Q因数'),
            ('DCR','R_ic','I_rms','DCR vs IC内阻 -> 回路电流'),
            ('R_d','C_t','f_res','阻尼电阻 vs 电容 -> 谐振频率'),
            ('L','R_ic','I_peak','电感 vs IC内阻 -> 峰值电流'),
            ('R_d','L','P_Rd','阻尼电阻 vs 电感 -> 电阻发热'),
            ('DCR','C_t','V_out_pp','DCR vs 电容 -> 驱动电压'),
        ]
        n_cols = 3
        n_rows = (len(pairs) + n_cols - 1) // n_cols
        fig, axes = plt.subplots(n_rows, n_cols, figsize=figsize)
        axes = np.atleast_1d(axes).flatten()
        fig.suptitle('双参数交叉分布 — 2D KDE 热力图', fontsize=15, fontweight='bold', y=0.99)

        inp = self._input_arrs()
        for i, (p1, p2, metric, tdesc) in enumerate(pairs):
            ax = axes[i]
            x = inp[p1]; y = inp[p2]; z = self._arr(metric)
            zf = np.isfinite(z)
            x, y, z = x[zf], y[zf], z[zf]

            if len(x) < 10:
                ax.set_visible(False)
                continue
            try:
                kde = stats.gaussian_kde(np.vstack([x, y]))
                xi, yi = np.mgrid[x.min():x.max():100j, y.min():y.max():100j]
                zi = kde(np.vstack([xi.ravel(), yi.ravel()])).reshape(xi.shape)
                ax.contourf(xi, yi, zi, levels=15, cmap='plasma', alpha=0.85)
                ax.contour(xi, yi, zi, levels=8, colors='black', linewidths=0.3, alpha=0.4)
            except Exception:
                ax.scatter(x, y, c=z, s=3, alpha=0.3, cmap='plasma')

            u1 = self.calc.components[p1]['unit']
            u2 = self.calc.components[p2]['unit']
            ax.set_xlabel(f'{self.calc.components[p1]["desc"]} ({u1})', fontsize=8)
            ax.set_ylabel(f'{self.calc.components[p2]["desc"]} ({u2})', fontsize=8)
            ax.set_title(tdesc, fontsize=9, fontweight='bold')

        for j in range(i + 1, len(axes)):
            axes[j].set_visible(False)

        fig.tight_layout(rect=[0, 0, 1, 0.98])
        if save_path:
            b, e = os.path.splitext(save_path)
            fig.savefig(f'{b}_2djoint{e or ".png"}')
            print(f'  [联合分布] -> {b}_2djoint{e or ".png"}')
        plt.close(fig)
        return fig

    # ------------------------------------------------------------------
    # 10. 综合仪表盘
    # ------------------------------------------------------------------
    def plot_dashboard_overview(self, save_path=None, figsize=(24, 18)):
        fig = plt.figure(figsize=figsize, constrained_layout=True)
        gs = GridSpec(4, 6, figure=fig)

        core = ['I_peak','V_out_pp','Q','f_res','Z_total',
                'P_board_total','P_ch_total','I_rms']

        # 左上: KDE 瀑布
        ax_wf = fig.add_subplot(gs[0:2, 0:3])
        for i, m in enumerate(core):
            d = self._arr(m)
            dc = d[np.isfinite(d)]
            kde = stats.gaussian_kde(dc)
            xr = np.linspace(dc.min(), dc.max(), 200)
            yk = kde(xr)
            yn = yk / yk.max()
            offset = (len(core) - 1 - i) * 1.2
            ax_wf.plot(xr, yn + offset, color=_PAL[i % len(_PAL)], linewidth=1.5)
            ax_wf.fill_between(xr, offset, yn + offset, alpha=0.15,
                               color=_PAL[i % len(_PAL)])
        ax_wf.set_yticks([(len(core) - 1 - i) * 1.2 + 0.5 for i in range(len(core))])
        ax_wf.set_yticklabels([_VM[m]['label'] for m in core][::-1], fontsize=8)
        ax_wf.set_title('核心指标 KDE 归一化瀑布', fontsize=11, fontweight='bold')
        ax_wf.set_xlabel('归一化值域')
        ax_wf.grid(axis='y', alpha=0.3, linestyle=':')

        # 右上: 方差贡献度
        ax_ct = fig.add_subplot(gs[0:2, 3:6])
        ik = list(self.calc.components.keys())
        ms = ['I_peak','V_out_pp','P_board_total']
        nd = {k: self.calc.components[k]['nom'] for k in self._data[0]}
        nr = self.calc._calc_state(nd)

        contrib = []
        for m in ms:
            row = []
            for p in ik:
                tl = self.calc.components[p]['tol']
                if tl < 1e-9:
                    row.append(0)
                    continue
                vp = nd[p] * (1 + tl)
                tv = dict(nd); tv[p] = vp
                rp = self.calc._calc_state(tv)
                row.append(abs(rp[m] - nr[m]) / abs(nr[m]) * 100 if abs(nr[m]) > 1e-15 else 0)
            s = sum(row)
            contrib.append([v / s * 100 if s > 1e-9 else 0 for v in row])

        xp = np.arange(len(ms))
        w = 0.6
        bottom = np.zeros(len(ms))
        for ii, p in enumerate(ik):
            vals = [contrib[j][ii] for j in range(len(ms))]
            ax_ct.bar(xp, vals, w, bottom=bottom, label=self.calc.components[p]['desc'],
                      color=_PAL[ii % len(_PAL)], edgecolor='white', linewidth=0.5)
            bottom += vals
        ax_ct.set_xticks(xp)
        ax_ct.set_xticklabels(ms, fontsize=9)
        ax_ct.set_ylabel('方差贡献度 (%)')
        ax_ct.set_title('各元件对关键输出方差贡献度', fontsize=11, fontweight='bold')
        ax_ct.legend(fontsize=6, ncol=3)

        # 中左: 统计卡片
        ax_st = fig.add_subplot(gs[2, 0:2])
        ax_st.axis('off')
        txt = '核心统计摘要\n' + '-' * 40 + '\n'
        for m in ['I_peak','V_out_pp','Q','P_board_total']:
            d = self._arr(m)
            info = _VM[m]
            txt += (f'{info["label"]}: '
                    f'P5={np.percentile(d,5):.3g}  '
                    f'u={np.mean(d):.3g}  '
                    f'P95={np.percentile(d,95):.3g}  '
                    f'sd={np.std(d):.3g} {info["unit"]}\n')
        ax_st.text(0, 1, txt, transform=ax_st.transAxes, fontsize=8,
                   va='top',
                   bbox=dict(boxstyle='round', facecolor='#ecf0f1', alpha=0.9))

        # 中中: 频率偏移
        ax_fr = fig.add_subplot(gs[2, 2:4])
        fres_a = np.array([r['f_res'] * 1e-3 for r in self._data[2]])
        df = fres_a - self.calc.f * 1e-3
        ax_fr.hist(df, bins=60, color='#3498db', edgecolor='white', alpha=0.7)
        ax_fr.axvline(0, color='red', linewidth=1.5, linestyle='--',
                       label=f'Target={self.calc.f*1e-3:.1f} kHz')
        ax_fr.set_xlabel('Delta f_res (kHz)')
        ax_fr.set_ylabel('Count')
        ax_fr.set_title(f'谐振频率偏移分布 (sd={np.std(df):.4f} kHz)', fontsize=10, fontweight='bold')
        ax_fr.legend(fontsize=7)
        ax_fr.grid(alpha=0.2)

        # 中右: 功率安全裕度
        ax_sf = fig.add_subplot(gs[2, 4:6])
        pd = self._arr('P_board_total')
        ax_sf.hist(pd, bins=60, color='#e67e22', edgecolor='white', alpha=0.7)
        ax_sf.axvline(36.0, color='red', linewidth=2, linestyle='--', label='36W limit')
        sp = np.sum(pd <= 36.0) / len(pd) * 100
        ax_sf.text(0.5, 0.95, f'Safe: {sp:.2f}% <= 36W\n'
                   f'u={np.mean(pd):.2f}W P95={np.percentile(pd,95):.2f}W',
                   transform=ax_sf.transAxes, fontsize=9, ha='center', va='top',
                   bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        ax_sf.set_xlabel('P_board_total (W)')
        ax_sf.set_ylabel('Count')
        ax_sf.set_title('整板功耗安全裕度', fontsize=10, fontweight='bold')
        ax_sf.legend(fontsize=7)
        ax_sf.grid(alpha=0.2)

        # 底部: 关键指标紧凑 KDE
        btm = ['I_peak','V_out_pp','P_board_total','Q','f_res','I_rms']
        for bi, m in enumerate(btm):
            ax = fig.add_subplot(gs[3, bi])
            d = self._arr(m)
            info = _VM[m]
            dc = d[np.isfinite(d)]
            kde = stats.gaussian_kde(dc)
            xr = np.linspace(dc.min(), dc.max(), 150)
            ax.plot(xr, kde(xr), color=_PAL[bi % len(_PAL)], linewidth=1.5)
            ax.fill_between(xr, kde(xr), alpha=0.25, color=_PAL[bi % len(_PAL)])
            ax.set_title(info['label'], fontsize=9, fontweight='bold')
            ax.set_xlabel(info['unit'], fontsize=7)
            ax.set_ylabel('Density', fontsize=7)
            ax.grid(alpha=0.2)

        fig.suptitle('超声相控阵 LC 谐振驱动 — 综合仪表盘', fontsize=16, fontweight='bold', y=1.02)

        if save_path:
            b, e = os.path.splitext(save_path)
            fig.savefig(f'{b}_dashboard{e or ".png"}')
            print(f'  [仪表盘] -> {b}_dashboard{e or ".png"}')
        plt.close(fig)
        return fig

    # ------------------------------------------------------------------
    # 总入口
    # ------------------------------------------------------------------
    def plot_all(self, output_dir='./visualization_output', prefix='LC_Analysis'):
        os.makedirs(output_dir, exist_ok=True)
        base = os.path.join(output_dir, prefix)

        print(f'\n{"="*60}')
        print(f'  开始生成可视化分析图表...')
        print(f'{"="*60}\n')

        methods = [
            (self.plot_kde_distributions,       'KDE概率密度曲线'),
            (self.plot_sensitivity_tornado,      '灵敏度龙卷风图'),
            (self.plot_correlation_heatmap,      '关联热力图'),
            (self.plot_parameter_sweep,          '单参数连续扫描'),
            (self.plot_scatter_matrix,           '散点矩阵图'),
            (self.plot_cdf_curves,               '累积分布函数(CDF)'),
            (self.plot_boxplot,                  '箱线图+小提琴图'),
            (self.plot_resonance_quality,        '谐振匹配质量评估'),
            (self.plot_2d_joint_distributions,   '双参数联合分布热力图'),
            (self.plot_dashboard_overview,       '综合仪表盘'),
        ]

        for fn, desc in methods:
            print(f'> 绘制 {desc}...')
            try:
                fn(save_path=base)
            except Exception as e:
                print(f'  [WARN] {desc} 失败: {e}')

        print(f'\n{"="*60}')
        print(f'  全部图表已生成至: {os.path.abspath(output_dir)}')
        print(f'{"="*60}')

    def quick_view(self, chart_name='dashboard', save_path=None):
        m = {
            'kde':          self.plot_kde_distributions,
            'sensitivity':  self.plot_sensitivity_tornado,
            'correlation':  self.plot_correlation_heatmap,
            'sweep':        self.plot_parameter_sweep,
            'scatter':      self.plot_scatter_matrix,
            'cdf':          self.plot_cdf_curves,
            'boxplot':      self.plot_boxplot,
            'quality':      self.plot_resonance_quality,
            'joint':        self.plot_2d_joint_distributions,
            'dashboard':    self.plot_dashboard_overview,
        }
        fn = m.get(chart_name)
        if fn is None:
            print(f'Unknown chart: {chart_name}. Available: {", ".join(m.keys())}')
            return
        if chart_name == 'kde':
            fn(save_path=save_path)
        else:
            fn(save_path=save_path)

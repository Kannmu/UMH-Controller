import math
import itertools
import statistics

class Color:
    PURPLE = '\033[95m'
    CYAN = '\033[96m'
    DARKCYAN = '\033[36m'
    BLUE = '\033[94m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    END = '\033[0m'

class UltrasonicResonanceCalculator:
    def __init__(self):
        # ==========================================
        # 1. 基础系统参数 (System Parameters)
        # ==========================================
        self.V_DD = 12.0          # V, BTL供电电压
        self.f = 40000.0          # Hz, 驱动频率
        self.N_ch = 60           # 通道数量
        
        # ==========================================
        # 2. 其它非公差参数 (Other Parameters)
        # ==========================================
        self.I_q = 0.003          # A, 驱动IC单通道静态电流 (约3mA)
        self.I_sat = 0.11        # A, 电感饱和电流
        self.f_SRF = 0.38e6       # Hz, 电感自谐振频率

        # ==========================================
        # 3. 电容、电阻和电感参数及公差 (Components & Tolerances)
        # ==========================================
        # 格式: '名称': {'nom': 标称值, 'tol': 公差比例, 'scale': 显示缩放, 'unit': 单位, 'desc': 描述}
        self.components = {
            'C_t':   {'nom': 2.1e-9,  'tol': 0.2, 'scale': 1e9,  'unit': 'nF', 'desc': '换能器静态电容'},
            'C_p':   {'nom': 0.0e-9,  'tol': 0.00, 'scale': 1e9,  'unit': 'nF', 'desc': '并联补偿电容'},
            'L':     {'nom': 6.8e-3,  'tol': 0.2, 'scale': 1000, 'unit': 'mH', 'desc': '匹配电感总值'},
            'DCR':   {'nom': 43,    'tol': 0.20, 'scale': 1,    'unit': 'Ω',  'desc': '电感直流电阻'},
            'R_d':   {'nom': 470.0,   'tol': 0.01, 'scale': 1,    'unit': 'Ω',  'desc': '外部串联阻尼电阻'},
            'R_ic':  {'nom': 6.0,     'tol': 0.20, 'scale': 1,    'unit': 'Ω',  'desc': '驱动IC导通内阻'},
            'R_rad': {'nom': 4000.0,    'tol': 0.20, 'scale': 1,    'unit': 'Ω',  'desc': '等效辐射阻抗'},
        }


    def _calc_state(self, vals):
        C_t = vals['C_t']
        C_p = vals['C_p']
        L = vals['L']
        DCR = vals['DCR']
        R_d = vals['R_d']
        R_ic = vals['R_ic']
        R_rad = vals['R_rad']

        V_in_peak = (4.0 / math.pi) * self.V_DD
        V_in_rms = V_in_peak / math.sqrt(2)
        R_total = R_ic + DCR + R_rad + R_d

        # 引入 f_SRF 影响后的本征有效电感
        if self.f < self.f_SRF:
            L_eff_srf = L / (1.0 - (self.f / self.f_SRF)**2)
        else:
            L_eff_srf = L * 10
            
        C_total = C_t + C_p
        X_C = 1.0 / (2 * math.pi * self.f * C_total)

        # 迭代求解真实稳态电流与电感 (不动点迭代)
        L_current = L_eff_srf
        for _ in range(20):
            X_L = 2 * math.pi * self.f * L_current
            X_total = X_L - X_C
            Z_total = math.sqrt(R_total**2 + X_total**2)
            
            I_rms_test = V_in_rms / Z_total
            I_peak_test = I_rms_test * math.sqrt(2)
            
            sat_ratio = min(I_peak_test / self.I_sat, 1.5)
            L_next = L_eff_srf * (1.0 - 0.2 * (sat_ratio ** 2))
            
            # 使用缓和因子(damping)避免数值振荡
            L_current = 0.5 * L_current + 0.5 * L_next 

        L_actual = L_current
        X_L = 2 * math.pi * self.f * L_actual
        X_total = X_L - X_C
        Z_total = math.sqrt(R_total**2 + X_total**2)
        
        f_res = 1.0 / (2 * math.pi * math.sqrt(L_actual * C_total))

        I_rms = V_in_rms / Z_total
        I_peak = I_rms * math.sqrt(2)
        
        V_out_rms = I_rms * X_C
        V_out_peak = V_out_rms * math.sqrt(2)
        V_out_pp = V_out_peak * 2.0
        
        Q = X_C / Z_total

        P_dyn_total = (I_rms**2) * R_total
        P_Rd = (I_rms**2) * R_d
        P_L = (I_rms**2) * DCR
        P_IC = (I_rms**2) * R_ic
        P_rad = (I_rms**2) * R_rad
        
        P_stat = self.V_DD * self.I_q
        P_ch_total = P_dyn_total + P_stat
        P_board_total = P_ch_total * self.N_ch
        
        return {
            'C_total': C_total,
            'L_actual': L_actual,
            'X_C': X_C,
            'X_L': X_L,
            'R_total': R_total,
            'Z_total': Z_total,
            'f_res': f_res,
            'Q': Q,
            'V_in_rms': V_in_rms,
            'I_rms': I_rms,
            'I_peak': I_peak,
            'V_out_pp': V_out_pp,
            'P_Rd': P_Rd,
            'P_L': P_L,
            'P_IC': P_IC,
            'P_rad': P_rad,
            'P_ch_total': P_ch_total,
            'P_board_total': P_board_total,
            'P_stat': P_stat
        }

    def _enumerate_results(self):
        keys = list(self.components.keys())
        param_cases = {}
        for k, v in self.components.items():
            nom = v['nom']
            tol = v['tol']
            param_cases[k] = [nom * (1 - tol), nom, nom * (1 + tol)]

        all_combinations = list(itertools.product(*[param_cases[k] for k in keys]))
        results = []
        input_matrix = []
        for combo in all_combinations:
            vals = dict(zip(keys, combo))
            res = self._calc_state(vals)
            results.append(res)
            input_matrix.append(combo)

        nom_vals = {k: self.components[k]['nom'] for k in keys}
        nom_result = self._calc_state(nom_vals)
        return keys, all_combinations, results, input_matrix, nom_vals, nom_result

    def analyze(self):
        keys, all_combinations, results, _, nom_vals, nom_result = self._enumerate_results()
            
        # 提取极值的辅助函数
        def get_min_max(metric):
            vals = [r[metric] for r in results]
            return min(vals), max(vals)

        # 计算标准差的辅助函数
        def get_std(metric):
            vals = [r[metric] for r in results]
            return statistics.stdev(vals) if len(vals) > 1 else 0.0

        # 辅助格式化函数
        def fmt(metric, scale, suffix):
            min_v, max_v = get_min_max(metric)
            nom_v = nom_result[metric]
            std_v = get_std(metric)
            return f"Min: {min_v*scale:7.2f} | Nom: {nom_v*scale:7.2f} | Max: {max_v*scale:7.2f} | Std: {std_v*scale:7.2f} {suffix}"

        print(f"{Color.CYAN}{'='*70}{Color.END}")
        print(f"{Color.BOLD}{Color.CYAN} 超声相控阵 LC 谐振驱动分析报告 (含公差极值分析){Color.END}")
        print(f"{Color.CYAN}{'='*70}{Color.END}")
        
        print(f"{Color.YELLOW}[元件参数与公差范围]{Color.END}")
        for k, v in self.components.items():
            scale = v['scale']
            unit = v['unit']
            min_val = v['nom'] * (1 - v['tol']) * scale
            nom_val = v['nom'] * scale
            max_val = v['nom'] * (1 + v['tol']) * scale
            idx = keys.index(k)
            c_vals = [combo[idx] * scale for combo in all_combinations]
            std_val = statistics.stdev(c_vals) if len(c_vals) > 1 else 0.0
            print(f"  {k:<5} ({v['desc']:<10}): Min: {min_val:7.2f} | Nom: {nom_val:7.2f} | Max: {max_val:7.2f} | Std: {std_val:7.2f} {unit} (±{v['tol']*100:.0f}%)")
        print("-" * 70)
        
        print(f"{Color.YELLOW}[匹配网络状态极值]{Color.END}")
        print(f"  总电容 (C_total)   : {Color.GREEN}{fmt('C_total', 1e9, 'nF')}{Color.END}")
        print(f"  动态有效电感 (L_eff): {Color.GREEN}{fmt('L_actual', 1000, 'mH')}{Color.END}")
        print(f"  容抗 (X_C)         : {Color.GREEN}{fmt('X_C', 1, 'Ω')}{Color.END}")
        print(f"  感抗 (X_L)         : {Color.GREEN}{fmt('X_L', 1, 'Ω')}{Color.END}")
        print(f"  电感SRF            : {Color.GREEN}{self.f_SRF/1000:.1f} kHz{Color.END}")
        if self.f >= self.f_SRF / 3:
            print(f"  {Color.RED}>>> 警告: 驱动频率接近或超过电感SRF的1/3，电感量已显著漂移！{Color.END}")
        print(f"  回路总电阻 (R_tot) : {Color.GREEN}{fmt('R_total', 1, 'Ω')}{Color.END}")
        print(f"  回路总阻抗 (|Z|)   : {Color.GREEN}{fmt('Z_total', 1, 'Ω')}{Color.END}")
        print(f"  物理谐振频率       : {Color.GREEN}{fmt('f_res', 1e-3, 'kHz')}{Color.END} (目标: {self.f/1000:.2f} kHz)")
        print(f"  回路品质因数 (Q)   : {Color.GREEN}{fmt('Q', 1, '')}{Color.END}")
        print("-" * 70)
        
        print(f"{Color.YELLOW}[电气与输出特性极值]{Color.END}")
        print(f"  输入基波有效值     : {Color.GREEN}{nom_result['V_in_rms']:.2f} V{Color.END} (恒定)")
        print(f"  回路电流 (I_rms)   : {Color.GREEN}{fmt('I_rms', 1000, 'mA')}{Color.END}")
        print(f"  回路峰值电流(I_peak): {Color.GREEN}{fmt('I_peak', 1000, 'mA')}{Color.END}")
        
        _, max_I_peak = get_min_max('I_peak')
        print(f"  电感饱和电流 (Isat): {Color.GREEN}{self.I_sat*1000:.2f} mA{Color.END}")
        if max_I_peak >= self.I_sat * 0.8:
            print(f"  {Color.RED}>>> 警告: 最恶劣情况下，峰值电流({max_I_peak*1000:.2f}mA)超过饱和电流({self.I_sat*1000:.2f}mA)的80%，有磁饱和风险！{Color.END}")
            
        print(f"  换能器驱动电压     : {Color.BOLD}{Color.GREEN}{fmt('V_out_pp', 1, 'Vpp')}{Color.END}")
        min_V_out_pp, _ = get_min_max('V_out_pp')
        if min_V_out_pp < 60:
            print(f"  {Color.RED}>>> 警告: 最恶劣情况下，换能器驱动电压不足 60Vpp (极小值为 {min_V_out_pp:.1f}Vpp)！{Color.END}")
        print("-" * 70)
        
        print(f"{Color.YELLOW}[发热与功耗分布极值 (单通道)]{Color.END}")
        print(f"  阻尼电阻发热 (R_d) : {Color.GREEN}{fmt('P_Rd', 1, 'W')}{Color.END}")
        _, max_P_Rd = get_min_max('P_Rd')
        if max_P_Rd > 0.125:
            print(f"  {Color.RED}>>> 建议: 最恶劣情况下阻尼电阻发热 > 0.125W (最大值为 {max_P_Rd:.3f}W)，请务必使用 0805 或更大型号封装。{Color.END}")
            
        print(f"  电感发热 (DCR)     : {Color.GREEN}{fmt('P_L', 1, 'W')}{Color.END}")
        print(f"  驱动IC发热 (动态)  : {Color.GREEN}{fmt('P_IC', 1, 'W')}{Color.END}")
        print(f"  超声有效辐射做功   : {Color.GREEN}{fmt('P_rad', 1, 'W')}{Color.END}")
        print(f"  单通道总功耗       : {Color.GREEN}{fmt('P_ch_total', 1, 'W')}{Color.END}")
        print("-" * 70)

        print(f"{Color.YELLOW}[整板系统功耗 ({self.N_ch}通道)]{Color.END}")
        print(f"  整板驱动总功耗     : {Color.BOLD}{Color.GREEN}{fmt('P_board_total', 1, 'W')}{Color.END}")
        _, max_P_board = get_min_max('P_board_total')
        if max_P_board > 36.0:
            print(f"  {Color.BOLD}{Color.RED}>>> 严重警告: 最恶劣情况下，整板功耗突破 36W 限制！(最大值为 {max_P_board:.2f}W){Color.END}")
            print(f"      {Color.RED}请尝试：1. 减小并联电容 C_p； 2. 增大阻尼电阻； 3. 使用更小公差的元件。{Color.END}")
        else:
            print(f"  {Color.GREEN}>>> 状态正常: 所有公差组合下，整板功耗均在 36W 预算范围内。{Color.END}")
        print(f"{Color.CYAN}{'='*70}{Color.END}")

if __name__ == "__main__":
    import argparse
    import os

    parser = argparse.ArgumentParser(description='超声相控阵 LC 谐振驱动分析 (含可视化)')
    parser.add_argument('--text', action='store_true', default=True,
                        help='输出文本分析报告 (默认开启)')
    parser.add_argument('--no-text', action='store_true',
                        help='跳过文本报告')
    parser.add_argument('--plot', action='store_true', default=False,
                        help='生成可视化图表 (默认开启)')
    parser.add_argument('--no-plot', action='store_true',
                        help='跳过可视化图表')
    parser.add_argument('--output', '-o', type=str, default='./visualization_output',
                        help='图表输出目录 (默认: ./visualization_output)')
    parser.add_argument('--prefix', '-p', type=str, default='LC_Analysis',
                        help='输出文件名前缀 (默认: LC_Analysis)')
    parser.add_argument('--chart', type=str, default='all',
                        choices=['kde','sensitivity','correlation','sweep','scatter',
                                 'cdf','boxplot','quality','joint','dashboard','all'],
                        help='仅生成指定图表 (默认: all 全部)')

    args = parser.parse_args()

    calc = UltrasonicResonanceCalculator()

    if not args.no_text and args.text:
        calc.analyze()
    
    if not args.no_plot and args.plot:
        try:
            from LC_Visualizer import UltrasonicResonanceVisualizer
            viz = UltrasonicResonanceVisualizer(calc)
            os.makedirs(args.output, exist_ok=True)
            base = os.path.join(args.output, args.prefix)

            if args.chart == 'all':
                viz.plot_all(output_dir=args.output, prefix=args.prefix)
            else:
                viz.quick_view(args.chart, save_path=base)
                print(f'\n图表已保存至: {os.path.abspath(args.output)}')
        except ImportError as e:
            print(f'{Color.RED}可视化模块导入失败: {e}')
            print(f'请确保 LC_Visualizer.py 在同一目录下，并已安装: matplotlib numpy scipy{Color.END}')
        except Exception as e:
            print(f'{Color.RED}可视化生成失败: {e}{Color.END}')

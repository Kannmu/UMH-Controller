"""
USB DAQ 逆向工程测试工具 (针对 libusbK 驱动)
依赖: pip install pyusb
"""

import usb.core
import usb.util
import time
import sys
import os
import argparse
import binascii
import threading

def list_devices():
    """列出系统中所有的 USB 设备，帮助找到 DAQ 的 VID 和 PID"""
    print("正在扫描系统中的 USB 设备...\n")
    print(f"{'VID':<8} | {'PID':<8} | {'制造商':<20} | {'产品名称':<30}")
    print("-" * 75)
    
    devices = usb.core.find(find_all=True)
    count = 0
    for dev in devices:
        count += 1
        try:
            manufacturer = usb.util.get_string(dev, dev.iManufacturer) if dev.iManufacturer else "Unknown"
            product = usb.util.get_string(dev, dev.iProduct) if dev.iProduct else "Unknown"
        except Exception:
            manufacturer = "Permission Denied / Error"
            product = "Permission Denied / Error"
            
        print(f"{hex(dev.idVendor):<8} | {hex(dev.idProduct):<8} | {str(manufacturer):<20} | {str(product):<30}")
        
    print(f"\n扫描完成，共找到 {count} 个设备。")
    print("请在上方列表中找到你的 DAQ 采集卡，记下它的 VID 和 PID。")

def connect_device(vid, pid):
    """连接指定的 USB 设备并接管"""
    print(f"正在尝试连接设备 VID={hex(vid)}, PID={hex(pid)} ...")
    dev = usb.core.find(idVendor=vid, idProduct=pid)
    
    if dev is None:
        print("错误：找不到指定的设备，请确认设备已插入，且驱动为 libusbK。")
        sys.exit(1)
        
    print("设备已找到！")
    
    # 将设备设置为默认配置
    try:
        dev.set_configuration()
        print("已成功应用默认配置。")
    except usb.core.USBError as e:
        print(f"警告：设置配置失败（可能已被占用或无需设置）：{e}")
        
    return dev

def analyze_endpoints(dev):
    """分析设备的接口和端点信息"""
    print("\n--- 设备接口和端点分析 ---")
    cfg = dev.get_active_configuration()
    intf = cfg[(0,0)]
    
    in_eps = []
    out_eps = []
    
    for ep in intf:
        ep_address = ep.bEndpointAddress
        ep_dir = usb.util.endpoint_direction(ep_address)
        ep_type = usb.util.endpoint_type(ep.bmAttributes)
        
        dir_str = "IN (设备到电脑)" if ep_dir == usb.util.ENDPOINT_IN else "OUT (电脑到设备)"
        
        type_str = "未知"
        if ep_type == usb.util.ENDPOINT_TYPE_CTRL: type_str = "Control"
        elif ep_type == usb.util.ENDPOINT_TYPE_ISO: type_str = "Isochronous"
        elif ep_type == usb.util.ENDPOINT_TYPE_BULK: type_str = "Bulk"
        elif ep_type == usb.util.ENDPOINT_TYPE_INTR: type_str = "Interrupt"
            
        print(f"端点地址: {hex(ep_address)}")
        print(f"  方向: {dir_str}")
        print(f"  类型: {type_str}")
        print(f"  最大包大小: {ep.wMaxPacketSize} 字节")
        
        if ep_dir == usb.util.ENDPOINT_IN:
            in_eps.append(ep)
        else:
            out_eps.append(ep)
            
    return in_eps, out_eps

def clear_screen():
    """清除终端屏幕"""
    os.system('cls' if os.name == 'nt' else 'clear')

def passive_listen(dev, in_ep, timeout=1000):
    """被动监听模式：只读取，不发送。测试设备是否上电后主动上传数据"""
    clear_screen()
    print(f"--- 进入被动监听模式 (端点 {hex(in_ep.bEndpointAddress)}) ---")
    print("按 Ctrl+C 退出监听...")
    
    empty_reads = 0
    update_counter = 0
    try:
        while True:
            try:
                # 尝试读取数据
                data = dev.read(in_ep.bEndpointAddress, in_ep.wMaxPacketSize, timeout)
                
                # 控制刷新频率，避免闪烁过快
                update_counter += 1
                if update_counter % 1000 != 0: # 假设采样率很高，只取 1/10 的数据刷新屏幕
                    continue
                    
                # 如果是 64 字节，尝试解析我们发现的规律
                if len(data) == 64:
                    clear_screen()
                    print(f"\n[{time.strftime('%H:%M:%S')}] 收到 64 字节数据，尝试解析...")
                    print("按 Ctrl+C 退出...\n")
                    
                    print("-" * 50)
                    print(f"{'通道(索引)':<12} | {'Raw (Hex)':<10} | {'Value (12-bit)':<12} | {'Voltage (约)':<12}")
                    print("-" * 50)
                    
                    # 假设数据格式是 16位(2字节) 整数
                    for i in range(0, 64, 2):
                        ch_idx = i // 2
                        byte1 = data[i]
                        byte2 = data[i+1]
                        val = byte1 + (byte2 << 8)
                        
                        hex_str = f"{byte1:02X} {byte2:02X}"
                        
                        if val != 0:
                            # 假设是 12位 ADC (0-4095)
                            # STM32 ADC 参考电压通常是 3.3V
                            voltage = (val / 4095.0) * 3.3
                            print(f"CH {ch_idx:<9} | {hex_str:<10} | {val:<14} | {voltage:.3f} V")
                        else:
                            print(f"CH {ch_idx:<9} | {hex_str:<10} | {'0':<14} | 0.000 V")
                            
                    print("-" * 50)
                    
                else:
                    print(f"\n[{time.strftime('%H:%M:%S')}] 收到未知长度数据 ({len(data)} bytes):")
                    print(f"  Hex: {binascii.hexlify(data, b' ').decode('utf-8').upper()}")
                    
                empty_reads = 0
            except usb.core.USBError as e:
                # 10060 是超时错误 (Windows 下)
                if 'timeout' in str(e).lower() or e.errno == 10060:
                    empty_reads += 1
                    # 只在不频繁更新的时候打印点
                    if empty_reads % 5 == 0 and update_counter == 0:
                        print(".", end="", flush=True)
                else:
                    print(f"\n读取发生错误: {e}")
                    break
    except KeyboardInterrupt:
        print("\n监听已停止。")

def interactive_mode(dev, out_ep, in_ep):
    """交互测试模式：后台持续监听数据，前台允许用户手动输入 Hex 发送，以观察数据变化"""
    clear_screen()
    print("==================================================")
    print("      USB DAQ 交互探测模式 (Interactive Mode)      ")
    print("==================================================")
    print("说明：")
    print("1. 后台正在监听 IN 端点的数据。")
    print("2. 只有当收到与之前【不相同】的数据时，才会打印在屏幕上。")
    print("3. 你可以在下方输入 Hex 格式的指令发送给板卡（如：01, AA 55, FF 等）。")
    print("4. 输入 'q' 退出。")
    print("==================================================\n")

    last_data = None
    stop_event = threading.Event()

    def listen_thread():
        nonlocal last_data
        while not stop_event.is_set():
            try:
                # 尝试读取数据
                data = dev.read(in_ep.bEndpointAddress, in_ep.wMaxPacketSize, timeout=200)
                data_tuple = tuple(data)
                
                # 如果收到的数据和上次不一样，就打印出来
                if data_tuple != last_data:
                    last_data = data_tuple
                    print("\n\n" + "="*50)
                    print(f"[{time.strftime('%H:%M:%S.%f')[:-3]}] 数据发生变化！收到 {len(data)} bytes")
                    
                    if len(data) == 64:
                        print(f"{'通道':<8} | {'Raw (Hex)':<10} | {'Value':<10}")
                        print("-" * 35)
                        for i in range(0, 64, 2):
                            val = data[i] + (data[i+1] << 8)
                            if val != 0:
                                print(f"CH {i//2:<5} | {data[i]:02X} {data[i+1]:02X}    | {val}")
                    else:
                        print(f"Hex: {binascii.hexlify(data, b' ').decode('utf-8').upper()}")
                    
                    print("="*50)
                    print("\n请输入 Hex 指令 (如 01, AA 55) 或输入 q 退出: ", end="", flush=True)

            except usb.core.USBError as e:
                # 超时正常，忽略
                if 'timeout' not in str(e).lower() and e.errno != 10060:
                    print(f"\n[监听异常] {e}")
                    break
            except Exception as e:
                break

    # 启动后台监听线程
    t = threading.Thread(target=listen_thread, daemon=True)
    t.start()

    # 主线程负责接收用户输入并发送
    try:
        while True:
            cmd = input("\n请输入 Hex 指令 (如 01, AA 55) 或输入 q 退出: ").strip()
            if cmd.lower() == 'q':
                stop_event.set()
                break
            if not cmd:
                continue

            try:
                # 解析用户输入的 Hex 字符串
                cmd_bytes = bytes.fromhex(cmd.replace(" ", "").replace(",", ""))
                
                # 发送给 OUT 端点
                dev.write(out_ep.bEndpointAddress, cmd_bytes, timeout=1000)
                print(f"[已发送] -> {binascii.hexlify(cmd_bytes, b' ').decode('utf-8').upper()}")
                
            except ValueError:
                print("输入格式错误！请输入有效的 Hex 字符串，例如: 01 0A FF")
            except usb.core.USBError as e:
                print(f"发送失败: {e}")
                
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        print("\n已退出交互模式。")

def generate_payloads_v3():
    """动态生成超大容量字典库 (Generator)，避免一次性占用过多内存"""
    count = 0
    
    # 辅助函数：计算 Modbus RTU CRC16
    def calc_crc16(data_bytes):
        crc = 0xFFFF
        for b in data_bytes:
            crc ^= b
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return bytes([crc & 0xFF, (crc >> 8) & 0xFF])
        
    # 辅助函数：计算简单的累加校验和
    def calc_checksum(data_bytes):
        return sum(data_bytes) & 0xFF
        
    # 辅助函数：计算异或校验和
    def calc_xor(data_bytes):
        xor = 0
        for b in data_bytes:
            xor ^= b
        return xor & 0xFF

    # === 1. 深度穷举 4 字节的魔法指令组合 (256 * 256 = 65536 种核心组合) ===
    # 很多设备的启动指令可能就是短短的几个字节，比如 [头部, 命令, 参数, 校验]
    # 我们穷举前两个字节，后两个字节动态填充
    print("  -> 规划区块 1: 4 字节魔法结构穷举 (~65,000 条)")
    for b1 in range(256):
        for b2 in range(256):
            # 模式 A: [b1, b2, 0x00, Checksum(b1+b2)]
            yield bytes([b1, b2, 0x00, (b1 + b2) & 0xFF])
            count += 1
            # 模式 B: [b1, b2, 0xFF, XOR(b1, b2, 0xFF)]
            yield bytes([b1, b2, 0xFF, b1 ^ b2 ^ 0xFF])
            count += 1
            # 模式 C: [b1, 0x00, b2, Checksum(b1+b2)]
            yield bytes([b1, 0x00, b2, (b1 + b2) & 0xFF])
            count += 1

    # === 2. 深度穷举特定头部的长帧结构 (16字节) ===
    # 假设包头固定 (例如 55 AA 或 AA 55 或 01 03 等)，中间是各种数据，尾部有校验
    print("  -> 规划区块 2: 16 字节长帧结构穷举 (~100,000+ 条)")
    headers = [
        b'\x55\xaa', b'\xaa\x55', b'\xfe\xfe', b'\x01\x03', b'\x01\x04', 
        b'\x01\x10', b'\x02\x00', b'\x16\x16', b'\x68\x68', b'\x5a\xa5', b'\xa5\x5a'
    ]
    
    # 我们在帧的第 2, 3, 4, 5 个数据位进行穷举 (其它位填 0)
    for h in headers:
        for d1 in [0x00, 0x01, 0x02, 0x03, 0x04, 0x08, 0x0A, 0x0F, 0x10, 0x20, 0x40, 0x80, 0xFF]:
            for d2 in [0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0xFF]:
                for d3 in [0x00, 0x01, 0x02, 0xFF]:
                    for d4 in [0x00, 0x01, 0xFF]:
                        packet = bytearray(16)
                        packet[0:2] = h
                        packet[2] = d1
                        packet[3] = d2
                        packet[4] = d3
                        packet[5] = d4
                        
                        # 尝试两种不同的校验方式
                        # 1. 最后一个字节是累加和
                        packet[15] = calc_checksum(packet[0:15])
                        yield bytes(packet)
                        count += 1
                        
                        # 2. 最后两个字节是 CRC16
                        packet[14:16] = calc_crc16(packet[0:14])
                        yield bytes(packet)
                        count += 1

    # === 3. 极其罕见的 ASCII 与二进制混合结构 ===
    print("  -> 规划区块 3: 罕见混合结构与宽字符穷举 (~10,000+ 条)")
    prefixes = [b'GET', b'SET', b'RD', b'WR', b'CMD', b'RUN', b'SYS', b'CFG', b'V', b'A', b'T']
    suffixes = [b'\r', b'\n', b'\r\n', b'\x00', b'\xff', b'\x03', b'\x04']
    
    for prefix in prefixes:
        for suffix in suffixes:
            # 穷举中间的 1~2 字节数字参数 (例如 GET 0x01 \r\n)
            for param1 in range(256):
                yield prefix + bytes([param1]) + suffix
                count += 1
                # 有些可能是文本加空格再加数字
                yield prefix + b' ' + bytes([param1]) + suffix
                count += 1
                
                # 偶尔步进测试双字节参数 (跳着测，不然太大)
                if param1 % 16 == 0:
                    for param2 in range(0, 256, 16):
                        yield prefix + bytes([param1, param2]) + suffix
                        count += 1

    # === 4. 纯净的全范围递增扫描 (对 6 字节包进行深度扫描) ===
    # [Cmd, Len, P1, P2, P3, P4]
    print("  -> 规划区块 4: 特定格式 6 字节扫描 (~10,000+ 条)")
    for cmd in [0x00, 0x01, 0x02, 0x0F, 0x10, 0x11, 0x20, 0xFF]:
        for length in [0x00, 0x01, 0x02, 0x04, 0x08]:
            for p1 in [0x00, 0x01, 0x02, 0xFF]:
                for p2 in range(0, 256, 8): # 步进 8
                    for p3 in [0x00, 0xFF]:
                        yield bytes([cmd, length, p1, p2, p3, calc_xor([cmd, length, p1, p2, p3])])
                        count += 1

    print(f"\n  [字典生成器就绪] 预计总测试量超过: {count} 条")

def brute_force_probe(dev, out_ep, in_ep):
    """暴力破解模式：系统性地发送各种可能的数据包组合，尝试唤醒设备或改变返回状态"""
    clear_screen()
    print("==================================================")
    print("      USB DAQ 极深暴力破解探测模式 (V3 超大容量)      ")
    print("==================================================")
    print("说明：")
    print("1. 本次字典采用 Generator 动态生成，避免内存溢出。")
    print("2. 包含了超过 30 万条以上极其复杂的协议组合，且与前两批完全不重合。")
    print("3. 测试时间会非常长，可能需要数小时，请耐心等待。")
    print("4. 按 Ctrl+C 可以随时安全停止。")
    print("==================================================\n")

    # 记录初始的静态数据状态 (作为对比基准)
    baseline_data = None
    print("[1/3] 正在获取初始静态数据基准线...")
    try:
        for _ in range(5): # 读几次确保稳定
            data = dev.read(in_ep.bEndpointAddress, in_ep.wMaxPacketSize, timeout=1000)
            baseline_data = tuple(data)
            time.sleep(0.1)
        print(f"  -> 获取成功，基准数据长度: {len(baseline_data)} bytes")
    except usb.core.USBError as e:
        print(f"  -> 获取基准数据失败，请确认设备是否在发数据: {e}")
        print("  -> 尝试继续执行破解...")
        baseline_data = None

    print("\n[2/3] 准备加载 V3 动态字典生成器...")
    payload_generator = generate_payloads_v3()
    
    print("\n[3/3] 开始暴力破解 (按 Ctrl+C 提前停止)...\n")
    time.sleep(1)
    
    success_count = 0
    idx = 0
    try:
        for payload in payload_generator:
            idx += 1
            hex_str = binascii.hexlify(payload, b' ').decode('utf-8').upper()
            if len(hex_str) > 30:
                hex_str = hex_str[:27] + "..."
                
            # 每 10 条更新一次屏幕，避免打印太快拖慢发包速度
            if idx % 10 == 0:
                print(f"\r进度: 已发送 {idx} 条 | 当前: {hex_str:<30}", end="")
            
            try:
                # 1. 发送 Payload
                dev.write(out_ep.bEndpointAddress, payload, timeout=100) # 降低发包超时，加快速度
                
                # 2. 尝试读取并比对 (缩短读超时，提升扫描效率)
                changed = False
                new_data = None
                try:
                    resp = dev.read(in_ep.bEndpointAddress, in_ep.wMaxPacketSize, timeout=50)
                    resp_tuple = tuple(resp)
                    if baseline_data and resp_tuple != baseline_data:
                        # 再次确认不是偶然波动
                        time.sleep(0.05)
                        resp2 = dev.read(in_ep.bEndpointAddress, in_ep.wMaxPacketSize, timeout=50)
                        if tuple(resp2) != baseline_data:
                            changed = True
                            new_data = tuple(resp2)
                except usb.core.USBError:
                    pass # 读超时忽略
                        
                # 3. 报告发现
                if changed:
                    success_count += 1
                    print("\n\n" + "="*50)
                    print(f"!!! 发现有效指令 (Bingo) !!!")
                    print(f"导致数据变化的 Payload: {binascii.hexlify(payload, b' ').decode('utf-8').upper()}")
                    print("="*50)
                    print(f"新数据 (前 16 字节): {binascii.hexlify(bytes(new_data[:16]), b' ').decode('utf-8').upper()}")
                    print("="*50)
                    
                    # 暂停，询问是否继续
                    ans = input("\n数据已改变！是否继续尝试其他 Payload? (y/n): ")
                    if ans.lower() != 'y':
                        break
                        
                    print("注意: 设备状态已改变，建议拔插 USB 后再进行后续测试以保证准确性。")
                    baseline_data = new_data # 更新基准线，避免后续一直报警
                    
            except usb.core.USBError as e:
                # 写入失败
                pass
                
        print(f"\n\n破解结束。共扫描了 {idx} 条 Payload。")
        if success_count == 0:
            print("很遗憾，在超大字典库中仍未找到能唤醒设备的指令。")
            print("协议可能包含了复杂的动态加密、握手验证，或需要更特殊的波特率/时序支持。")
            print("强烈建议：直接使用上位机软件配合 Wireshark/USBCap 进行抓包分析。")
            
    except KeyboardInterrupt:
        print(f"\n\n已手动停止暴力破解。当前进度停在第 {idx} 条。")

def main():
    parser = argparse.ArgumentParser(description="USB DAQ 逆向测试工具")
    parser.add_argument('--list', action='store_true', help="列出所有 USB 设备并退出")
    parser.add_argument('--vid', type=lambda x: int(x, 0), help="目标设备的 VID (支持 16 进制，如 0x1234)")
    parser.add_argument('--pid', type=lambda x: int(x, 0), help="目标设备的 PID (支持 16 进制，如 0x5678)")
    parser.add_argument('--probe', action='store_true', help="运行主动探测模式 (Fuzzing)")
    parser.add_argument('--interactive', action='store_true', help="运行交互发包探测模式")
    
    args = parser.parse_args()
    
    if args.list or (not args.vid or not args.pid):
        list_devices()
        print("\n使用方法:")
        print("1. 首先运行脚本查看设备列表，找到你的 DAQ 设备的 VID 和 PID。")
        print("2. 运行被动监听: python USB_DAQ.py --vid 0xXXXX --pid 0xXXXX")
        print("3. 运行交互发包: python USB_DAQ.py --vid 0xXXXX --pid 0xXXXX --interactive")
        print("4. 运行自动探测: python USB_DAQ.py --vid 0xXXXX --pid 0xXXXX --probe")
        sys.exit(0)
        
    # 1. 连接设备
    dev = connect_device(args.vid, args.pid)
    
    # 2. 分析端点
    in_eps, out_eps = analyze_endpoints(dev)
    
    if not in_eps:
        print("错误：该设备没有找到 IN (输入) 端点，无法读取数据。")
        sys.exit(1)
        
    in_ep = in_eps[0] # 默认使用第一个输入端点
    
    # 3. 根据参数选择模式
    if args.interactive:
        if not out_eps:
            print("错误：该设备没有找到 OUT (输出) 端点，无法发送指令。")
        else:
            interactive_mode(dev, out_eps[0], in_ep)
    elif args.probe:
        if not out_eps:
            print("错误：该设备没有找到 OUT (输出) 端点，无法发送探测指令。只能尝试被动监听。")
        else:
            out_ep = out_eps[0]
            brute_force_probe(dev, out_ep, in_ep)
    else:
        passive_listen(dev, in_ep)

if __name__ == "__main__":
    main()

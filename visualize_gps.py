import serial
import serial.tools.list_ports
import utm  # 用于将经纬度转换为UTM坐标（米为单位）
from datetime import datetime
import threading
import os
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.animation import FuncAnimation
import numpy as np
from math import radians, sin, cos, sqrt, atan2
import argparse
plt.rcParams['font.family'] = 'Songti SC'  # 设置中文字体
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示异常


# 文本界面美化
class TerminalColors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

# 清屏函数
def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

# 程序标题
def print_header():
    clear_screen()
    print(f"{TerminalColors.HEADER}{TerminalColors.BOLD}")
    print("==================================================")
    print("   GPS数据记录与实时轨迹可视化工具")
    print("==================================================")
    print(f"{TerminalColors.ENDC}")

# 列出所有串口设备
def list_serial_devices():
    ports = serial.tools.list_ports.comports()
    
    if not ports:
        print(f"{TerminalColors.FAIL}错误: 没有检测到任何串口设备{TerminalColors.ENDC}")
        return []
    
    print(f"{TerminalColors.OKBLUE}可用的串口设备:{TerminalColors.ENDC}")
    for i, port in enumerate(ports):
        vid_pid = f"{port.vid:04X}:{port.pid:04X}" if port.vid and port.pid else "未知"
        print(f"  {i+1}. {port.device} - {port.description} (HWID: {vid_pid})")
    
    return ports

# 用户选择设备
def select_device():
    ports = list_serial_devices()
    
    if not ports:
        return None
    
    while True:
        try:
            choice = input("选择设备 (输入编号): ")
            if choice.strip() == "":
                return None
            idx = int(choice) - 1
            if 0 <= idx < len(ports):
                return ports[idx].device
            print(f"{TerminalColors.WARNING}无效选择，请重新输入{TerminalColors.ENDC}")
        except ValueError:
            print(f"{TerminalColors.WARNING}请输入数字{TerminalColors.ENDC}")

# 解析NMEA语句中的经纬度
def parse_nmea_lat_lon(nmea_line):
    try:
        if not nmea_line.startswith('$'):
            return None, None
        
        parts = nmea_line.split(',')
        
        # 解析GGA和RMC语句
        if parts[0] in ['$GNGGA', '$GPGGA', '$GNRMC', '$GPRMC']:
            # GGA语句：纬度在索引2，经度在索引4
            if 'GGA' in parts[0]:
                lat_str = parts[2]
                lat_dir = parts[3]
                lon_str = parts[4]
                lon_dir = parts[5]
            # RMC语句：纬度在索引3，经度在索引5
            elif 'RMC' in parts[0]:
                lat_str = parts[3]
                lat_dir = parts[4]
                lon_str = parts[5]
                lon_dir = parts[6]
            else:
                return None, None

            # 解析纬度：格式为DDMM.MMMMMM
            lat_deg = float(lat_str[:2])
            lat_min = float(lat_str[2:])
            latitude = lat_deg + lat_min / 60.0
            if lat_dir == 'S':
                latitude = -latitude

            # 解析经度：格式为DDDMM.MMMMMM
            lon_deg = float(lon_str[:3])
            lon_min = float(lon_str[3:])
            longitude = lon_deg + lon_min / 60.0
            if lon_dir == 'W':
                longitude = -longitude

            return latitude, longitude
        return None, None
    except (ValueError, IndexError):
        return None, None

# 计算两点间距离（米）
def haversine(lat1, lon1, lat2, lon2):
    R = 6371  # 地球半径(km)
    
    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)
    lat1 = radians(lat1)
    lat2 = radians(lat2)
    
    a = sin(dlat/2)**2 + cos(lat1)*cos(lat2)*sin(dlon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    
    return R * c * 1000  # 转换为米

# GPS数据记录和可视化类
class GPSVisualizer:
    def __init__(self, start_lat_on_map, start_lon_on_map, device, baud=115200, max_points=1200, sim=False):
        self.max_points = max_points # 可视化数据长度
        self.device = device
        self.baud = baud
        # self.save_mp4 = save_mp4
        self.sim = sim  # 是否模拟数据
        self.log_filename = f"./gps_logs/gps_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        self.lat_lon_points = list()  # 存储经纬度点
        self.utm_points = list() # 存储UTM坐标点
        self.altitude = list() # 存储海拔高度
        self.current_speed = 0
        self.satellite_num = 0
        self.utm_moving_distance = 0
        self.lat_lon_moving_distance = 0
        self.lock = threading.Lock()
        self.running = True
        self.start_lat_on_map = start_lat_on_map
        self.start_lon_on_map = start_lon_on_map
        # 将起始点转换为UTM坐标
        self.start_easting_on_map, self.start_northing_on_map, _, __ = utm.from_latlon(self.start_lat_on_map, self.start_lon_on_map)
        
        # 创建图形和坐标轴
        self.fig, (self.ax1, self.ax2) = plt.subplots(2,1,figsize=(6, 12))
        self.trajectory_line, = self.ax1.plot([], [], 'b-', linewidth=1.5, alpha=0.7, label="轨迹线")  # 轨迹线
        self.start_point_on_map, = self.ax1.plot([], [], 'go', markersize=8, label='地图起始点')  # 起始点
        self.current_point, = self.ax1.plot([], [], 'ro', markersize=6, label="当前点")     # 当前点
        self.altitude_line, = self.ax2.plot([], [], 'g-', linewidth=1.5, alpha=0.7, label="海拔线")  # 海拔线
        
        # 设置初始坐标轴
        self.ax1.set_xlabel('东向距离 (米)')
        self.ax1.set_ylabel('北向距离 (米)')
        self.ax1.set_title(f'实时GPS轨迹 (最近{self.max_points}个点)')
        self.ax1.grid(True, linestyle='--', alpha=0.6)
        # 设置坐标轴比例相同
        self.ax1.set_aspect('equal', adjustable='box')
        self.ax1.legend(loc='upper right')

        # 设置第二个坐标轴显示海拔信息
        self.ax2.set_xlabel('最近点数 (个)')
        self.ax2.set_ylabel('海拔高度 (米)')
        self.ax2.set_title('海拔高度变化')
        self.ax2.grid(True, linestyle='--', alpha=0.6)
        self.ax2.legend(loc='upper right')
        
        # 文本信息显示
        self.info_text = self.ax1.text(0.02, 0.98, '', 
                                      transform=self.ax1.transAxes,
                                      verticalalignment='top',
                                      bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
    
    def gps_logger(self):
        """GPS数据记录线程"""
        if self.sim:
            print(f"{TerminalColors.OKBLUE}模拟数据模式已启用，实际GPS数据将不会被记录{TerminalColors.ENDC}")
            # 模拟数据生成
            import time
            import random
            
            print(f"{TerminalColors.OKGREEN}模拟设备开始记录数据...{TerminalColors.ENDC}")
            while self.running:
                # 模拟经纬度数据
                last_lat, last_lon = self.lat_lon_points[-1] if self.lat_lon_points else (self.start_lat_on_map, self.start_lon_on_map)
                lat = last_lat + random.uniform(-0.00001, 0.00001)
                lon = last_lon + random.uniform(-0.00001, 0.00001)
                easting, northing, zone_number, zone_letter = utm.from_latlon(lat, lon)
                # 计算移动距离
                if len(self.utm_points) > 0 and len(self.lat_lon_points) > 0:
                    last_easting, last_northing = self.utm_points[-1]
                    utm_distance = np.sqrt((easting - last_easting) ** 2 + (northing - last_northing) ** 2)
                    self.utm_moving_distance += utm_distance
                self.lat_lon_points.append((lat, lon))
                self.utm_points.append((easting, northing))

                # 模拟海拔和卫星数
                last_altitude = self.altitude[-1] if self.altitude else 0
                altitude = last_altitude + random.uniform(-1, 1)
                satellite_num = random.randint(1, 12)
                current_speed = random.uniform(0, 10)  # 模拟速度，单位为米/秒

                self.altitude.append(altitude)
                self.satellite_num = satellite_num
                self.current_speed = current_speed

                time.sleep(0.2)

        else:
            try:
                with serial.Serial(self.device, self.baud, timeout=1) as ser:
                    print(f"{TerminalColors.OKGREEN}设备 {self.device} 开始记录, 波特率: {self.baud}{TerminalColors.ENDC}")
                    print(f"{TerminalColors.OKGREEN}数据保存到: {self.log_filename}{TerminalColors.ENDC}")
                    
                    ser.flushInput()
                    
                    while self.running:
                        try:
                            line = ser.readline().decode('ascii', errors='replace').strip()
                            if not line:
                                continue
                            
                            # 记录原始数据
                            timestamp = datetime.now()
                            timestamp_str = timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                            
                            # 解析位置数据
                            lat, lon = parse_nmea_lat_lon(line)
                            easting, northing, zone_number, zone_letter = utm.from_latlon(lat, lon)

                            # 计算移动距离
                            if len(self.utm_points) > 0 and len(self.lat_lon_points) > 0:
                                last_easting, last_northing = self.utm_points[-1]
                                last_lat, last_lon = self.lat_lon_points[-1]
                                utm_distance = np.sqrt((easting - last_easting) ** 2 + (northing - last_northing) ** 2)
                                lat_lon_distance = haversine(lat, lon, last_lat, last_lon)
                                self.utm_moving_distance += utm_distance
                                self.lat_lon_moving_distance += lat_lon_distance

                            self.utm_points.append((easting, northing))
                            self.lat_lon_points.append((lat, lon))
                            
                            # 解析海拔和卫星数量（来自GGA语句）
                            if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
                                parts = line.split(',')
                                if len(parts) > 9:
                                    try:
                                        if parts[7]:
                                            self.satellite_num = int(parts[7])
                                        if parts[9]:
                                            self.altitude.append(float(parts[9]))
                                    except ValueError:
                                        pass

                            # 解析速度（来自RMC语句）
                            if line.startswith('$GNRMC') or line.startswith('$GPRMC'):
                                parts = line.split(',')
                                if len(parts) > 7:
                                    try:
                                        # 节转换为m/s
                                        self.current_speed = float(parts[7]) * 0.514444  # 节转换为米每秒
                                    except ValueError:
                                        pass

                            # 保存数据
                            with self.lock:
                                # 写入日志文件
                                # print(f"Current Line: [{timestamp_str}] {line}")
                                with open(self.log_filename, 'a') as f:
                                    f.write(f"[{timestamp_str}] {line}\n")
                                    f.flush()  # 确保实时写入
                        
                        except UnicodeDecodeError:
                            continue
                        except serial.SerialException as e:
                            print(f"{TerminalColors.FAIL}串口错误: {e}{TerminalColors.ENDC}")
                            break
                        except Exception as e:
                            print(f"{TerminalColors.FAIL}发生错误: {e}{TerminalColors.ENDC}")
                            break
            except Exception as e:
                print(f"{TerminalColors.FAIL}初始化失败: {e}{TerminalColors.ENDC}")
    
    def update_plot(self, frame):
        """更新图表"""
        with self.lock:
            if len(self.utm_points) == 0 or len(self.lat_lon_points) == 0:
                print(f"{TerminalColors.WARNING}没有可用的UTM或经纬度数据，等待数据更新...{TerminalColors.ENDC}")
                return self.trajectory_line, self.start_point_on_map, self.current_point, self.altitude_line, self.info_text
            # print(f"{TerminalColors.OKBLUE}更新图表...{TerminalColors.ENDC}")
            # 获取最近的位置数据
            x_vals, y_vals = zip(*self.utm_points[-self.max_points:])
            
            # 计算轨迹范围
            x_min, x_max = min(list(x_vals)+[self.start_easting_on_map]), max(list(x_vals)+[self.start_easting_on_map])
            y_min, y_max = min(list(y_vals)+[self.start_northing_on_map]), max(list(y_vals)+[self.start_northing_on_map])
            ref_x, ref_y = (x_max + x_min) / 2, (y_max + y_min) / 2
            
            # 计算最大偏移范围
            x_range = x_max - x_min
            y_range = y_max - y_min
            max_range = max(x_range, y_range, 0.1)  # 确保至少使用最小刻度

            xs = np.array(x_vals) - ref_x
            ys = np.array(y_vals) - ref_y
            self.start_point_on_map.set_data([self.start_easting_on_map - ref_x], [self.start_northing_on_map - ref_y])
            self.current_point.set_data([xs[-1]], [ys[-1]])
            self.trajectory_line.set_data(xs, ys)

            # 设置坐标轴范围（带10%边界）
            margin = max_range * 0.1
            self.ax1.set_xlim(-max_range/2 - margin, max_range/2 + margin)
            self.ax1.set_ylim(-max_range/2 - margin, max_range/2 + margin)
            

            # 更新信息文本
            self.info_text.set_text(
                f"卫星数: {self.satellite_num}\n"
                f"UTM移动距离: {self.utm_moving_distance:.2f} 米\n"
                f"当前速度: {self.current_speed:.2f} 米/秒\n"
                f"点数: {min(self.max_points, len(self.utm_points))}\n"
            )

            # 更新海拔高度图
            if len(self.altitude) > 0:
                altitude_vals = self.altitude[-self.max_points:]
                self.altitude_line.set_data(range(len(altitude_vals)), altitude_vals)
                alt_range = max(altitude_vals) - min(altitude_vals)
                self.ax2.set_xlim(0, max(1, len(altitude_vals) - 1))
                self.ax2.set_ylim(min(altitude_vals)-max(0.1, 0.1*alt_range), max(altitude_vals)+max(0.1, 0.1*alt_range))
                # self.ax2.set_ylim(min(altitude_vals) - 10, max(altitude_vals) + 10)
            else:
                print(f"{TerminalColors.WARNING}没有可用的海拔数据{TerminalColors.ENDC}")
            
            return self.trajectory_line, self.start_point_on_map, self.current_point, self.altitude_line, self.info_text

    def start(self):
        """启动GPS记录和可视化"""
        # 启动记录线程
        self.thread = threading.Thread(target=self.gps_logger, daemon=True)
        self.thread.start()
        
        # 启动动画
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=200, blit=False, cache_frame_data=False)
        
        # 显示图表
        plt.tight_layout()
        plt.show()
        
        # 图表关闭时停止记录
        self.running = False
        self.thread.join(timeout=1.0)

        # 输入终止点位置
        end_lat_on_map = float(input("请输入终止点纬度: "))
        end_lon_on_map = float(input("请输入终止点经度: "))
        end_easting_on_map, end_northing_on_map, _, _ = utm.from_latlon(end_lat_on_map, end_lon_on_map)
        
        # 保存总轨迹到PNG文件 
        fig1, ax1 = plt.subplots(figsize=(10, 8))
        x_vals, y_vals = zip(*self.utm_points)
        x_min, x_max = min((list(x_vals))+[self.start_easting_on_map, end_easting_on_map]), max(list(x_vals)+[self.start_easting_on_map, end_easting_on_map])
        y_min, y_max = min(list(y_vals)+[self.start_northing_on_map, end_northing_on_map]), max(list(y_vals)+[self.start_northing_on_map, end_northing_on_map])
        ref_x, ref_y = (x_max + x_min) / 2, (y_max + y_min) / 2
        half_range = (max(x_max - x_min, y_max - y_min)) / 2
        xs = np.array(x_vals) - ref_x
        ys = np.array(y_vals) - ref_y

        ax1.plot(xs, ys, 'b-', linewidth=1.5, alpha=0.7, label="总轨迹线")
        ax1.set_xlabel('东向距离 (米)')
        ax1.set_ylabel('北向距离 (米)')
        ax1.set_title('总GPS轨迹')
        ax1.grid(True, linestyle='--', alpha=0.6)

        # 绘制地图起点和终点
        ax1.plot([self.start_easting_on_map-ref_x, end_easting_on_map-ref_x],[self.start_northing_on_map-ref_y, end_northing_on_map-ref_y], 'ro--', markersize=4, alpha=0.5, label='地图参考起点/终点')
        
        ref_moving_distance = np.sqrt((end_easting_on_map - self.start_easting_on_map) ** 2 + (end_northing_on_map - self.start_northing_on_map) ** 2)
            
        # 计算轨迹范围
        start_utm = self.utm_points[0] if self.utm_points else (0, 0)
        end_utm = self.utm_points[-1] if self.utm_points else (0, 0)
        ax1.plot([start_utm[0]-ref_x], [start_utm[1]-ref_y], 'go', markersize=4, alpha=0.5, label='起点')
        ax1.plot([end_utm[0]-ref_x], [end_utm[1]-ref_y], 'ko', markersize=4, alpha=0.5, label='终点')
        ax1.set_xlim(-half_range * 1.1, half_range * 1.1)
        ax1.set_ylim(-half_range * 1.1, half_range * 1.1)
        ax1.legend(loc='upper right')
        ax1.text(0.02, 0.98, f"总移动距离: {self.utm_moving_distance:.2f} 米\n"
                            f"参考点/线移动距离: {ref_moving_distance:.2f} 米\n"
                            f"地图起点经纬度: ({self.start_lat_on_map:.7f}, {self.start_lon_on_map:.7f})\n"
                            f"地图终点经纬度: ({end_lat_on_map:.7f}, {end_lon_on_map:.7f})",
                transform=ax1.transAxes, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        # 横纵坐标轴比例相同
        ax1.set_aspect('equal', adjustable='box')
        plt.tight_layout()
        plt.show()

        png_filename = f"./gps_pngs/gps_track_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        fig1.savefig(png_filename, dpi=300)
        print(f"{TerminalColors.OKGREEN}图表已保存到: {png_filename}{TerminalColors.ENDC}")

        # 画出起点和终点的放大图
        fig2, (ax2, ax3) = plt.subplots(2, 1, figsize=(6, 13))

        ax2.plot(np.array(x_vals) - self.start_easting_on_map, np.array(y_vals) - self.start_northing_on_map, 'b-', linewidth=1.5, alpha=0.7, label="总轨迹线")
        ax2.set_xlabel('东向距离 (米)')
        ax2.set_ylabel('北向距离 (米)')
        ax2.set_title('GPS轨迹放大图 (起点)')
        ax2.grid(True, linestyle='--', alpha=0.6)
        ax2.plot([0], [0], 'go', markersize=4, alpha=0.5, label='起点')
        ax2.set_xlim(-10, 10)
        ax2.set_ylim(-10, 10)
        ax2.legend(loc='upper right')
        ax2.set_aspect('equal', adjustable='box')
        # ax2.text(0.02, 0.98, f"起点经纬度: ({self.start_lat_on_map:.7f}, {self.start_lon_on_map:.7f})",
        #          transform=ax2.transAxes, verticalalignment='top',
        #          bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        ax3.plot(np.array(x_vals) - end_easting_on_map, np.array(y_vals) - end_northing_on_map, 'b-', linewidth=1.5, alpha=0.7, label="总轨迹线")
        ax3.set_xlabel('东向距离 (米)')
        ax3.set_ylabel('北向距离 (米)')
        ax3.set_title('GPS轨迹放大图 (终点)')
        ax3.grid(True, linestyle='--', alpha=0.6)
        ax3.plot([0], [0], 'ro', markersize=4, alpha=0.5, label='终点')
        ax3.set_xlim(-10, 10)
        ax3.set_ylim(-10, 10)
        ax3.legend(loc='upper right')
        ax3.set_aspect('equal', adjustable='box')
        # ax3.text(0.02, 0.98, f"终点经纬度: ({end_lat_on_map:.7f}, {end_lon_on_map:.7f})",
        #          transform=ax3.transAxes, verticalalignment='top',
        #             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        plt.tight_layout()
        plt.show()
        png_filename_zoom = f"./gps_pngs/gps_track_zoom_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        fig2.savefig(png_filename_zoom, dpi=300)

        # 保存动画为mp4文件
        # if self.save_mp4:
        #     if not os.path.exists('./gps_mp4'):
        #         os.makedirs('./gps_mp4')
        #     print(f"{TerminalColors.OKBLUE}正在保存动画为MP4文件...{TerminalColors.ENDC}")
        #     writemp4 = animation.FFMpegWriter(fps=5)
        #     self.ani.save(f"./gps_mp4/gps_animation_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4", writer=writemp4)
        #     print(f"{TerminalColors.OKGREEN}动画已保存到: ./gps_mp4/gps_animation_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4{TerminalColors.ENDC}")

# 主程序
def main(lat_on_map, lon_on_map, baud=115200, max_points=1200, sim=False):
    print_header()
    
    # 选择设备
    if not sim:
        device = select_device()
        if not device:
            print(f"{TerminalColors.FAIL}未选择设备，程序退出{TerminalColors.ENDC}")
            return
    else:
        device = None
    
    import os
    if not os.path.exists('gps_logs'):
        os.makedirs('gps_logs')
    if not os.path.exists('gps_pngs'):
        os.makedirs('gps_pngs')

    # 创建并启动可视化器
    visualizer = GPSVisualizer(lat_on_map, lon_on_map, device, baud=baud, max_points=max_points, sim=sim)
    visualizer.start()

if __name__ == "__main__":
    # 运行时输入在地图上的起始点经纬度
    parser = argparse.ArgumentParser(description='GPS数据记录与可视化工具')
    parser.add_argument('--lat', type=float, required=True, help='起始点纬度')
    parser.add_argument('--lon', type=float, required=True, help='起始点经度')
    parser.add_argument('--baud', type=int, default=115200, help='串口波特率 (默认: 115200)')
    parser.add_argument('--max_points', type=int, default=1200, help='可视化最大点数 (默认: 1200)')
    # parser.add_argument('--save_mp4', action="store_true", help='是否保存动画为MP4文件 (默认: 不保存)')
    parser.add_argument('--sim', action='store_true', help='是否模拟数据 (默认: 不模拟)')
    args = parser.parse_args()
    lat_on_map = args.lat
    lon_on_map = args.lon
    baud = args.baud
    max_points = args.max_points
    # save_mp4 = args.save_mp4
    sim = args.sim
    if sim:
        print(f"{TerminalColors.WARNING}模拟数据模式已启用，实际GPS数据将不会被记录{TerminalColors.ENDC}")
        baud = 9600  # 模拟数据波特率设置为较低值
    else:
        print(f"{TerminalColors.OKGREEN}实际GPS数据记录模式已启用{TerminalColors.ENDC}")
    main(lat_on_map, lon_on_map, baud, max_points, sim)
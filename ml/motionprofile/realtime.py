import subprocess
import threading
import sys
import display
import collections

cur_info = collections.defaultdict(float)

def run():
    prospath = {
        "darwin": "/Users/leon.zhu/Library/Application Support/Code/User/globalStorage/sigbots.pros/install/pros-cli-macos/pros",
        "win32": "C:/Users/leon/AppData/Roaming/Code/User/globalStorage/sigbots.pros/install/pros-cli-windows/pros"
    }

    proc = subprocess.Popen([
        prospath[sys.platform],
        "terminal"
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    def updater():
        while True:
            line = proc.stdout.readline().decode("utf-8").strip()
            line = line.split(",")
            print(line)
            if len(line) == 15:
                cur_info['time'] = float(line[0]) / 1e6
                cur_info["x"] = float(line[1])
                cur_info["y"] = float(line[2])
                cur_info['theta'] = float(line[3])
                cur_info['left_voltage'] = float(line[4])
                cur_info['right_voltage'] = float(line[5])
                cur_info['left_velocity'] = float(line[6])
                cur_info['right_velocity'] = float(line[7])
                cur_info['left_actual_voltage'] = float(line[8])
                cur_info['right_actual_voltage'] = float(line[9])
                cur_info['left_actual_velocity'] = float(line[10])
                cur_info['right_actual_velocity'] = float(line[11])
                cur_info['clamp'] = float(line[12])
                cur_info['intake'] = float(line[13])
                cur_info['conveyor'] = float(line[14])

    lidarthread = threading.Thread(target=updater, daemon=True)
    lidarthread.start()

if __name__ == "__main__":
    run()

    disp = display.Display()
    disp.start()

    while True:
        disp.update(cur_info)
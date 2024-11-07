import numpy as np
import threading
import subprocess
import json
import sys

listlock = threading.Lock()
lidarpoints = []

def project_quaternion(quaternion, dist):
    x = dist * (1 - 2 * (quaternion[1]**2 + quaternion[2]**2))
    y = dist * 2 * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3])
    z = -dist * 2 * (quaternion[0] * quaternion[2] - quaternion[1] * quaternion[3])

    return x, y, z

def run():
    global lidarpoints, listlock

    prospath = {
        "macos": "/Users/leon.zhu/Library/Application Support/Code/User/globalStorage/sigbots.pros/install/pros-cli-macos/pros",
        "win32": "C:/Users/leon/AppData/Roaming/Code/User/globalStorage/sigbots.pros/install/pros-cli-windows/pros"
    }

    proc = subprocess.Popen([
        prospath[sys.platform],
        "terminal"
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    def lidar():
        global lidarpoints
        while True:
            line = proc.stdout.readline().decode("utf-8").strip()
            if "LIDAR" in line:
                line = line.split("LIDAR: ")[1]
                line = json.loads(line)
                if line['confidence'] >= 10:
                    relp = np.array(project_quaternion(
                        line['quaternion'],
                        line["distance"]
                    ))
                    globp = relp

                    listlock.acquire()
                    lidarpoints.append(globp)
                    listlock.release()

    lidarthread = threading.Thread(target=lidar, daemon=True)
    lidarthread.start()
import platform
import psutil

def get_cpu_temp():
    system = platform.system()
    if system == "Windows":
        return 0
    else:
        temps = psutil.sensors_temperatures()
        if not temps:
            return 0
        for key in ('coretemp', 'k10temp', 'cpu_thermal', 'acpitz'):
            if key in temps:
                return temps[key][0].current
        return round(next(iter(temps.values()))[0].current,2)
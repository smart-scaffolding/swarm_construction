import os
from multiprocessing import Pool
import subprocess

processes = (
    "components/structure/main.py",
    'components/robot/main.py -i "ROBOT_1"',
    'components/robot/main.py -i "ROBOT_2"',
    'components/robot/main.py -i "ROBOT_3"',
    'components/robot/main.py -i "ROBOT_4"',
)


def run_process(process):
    cwd = os.getcwd()
    command = "python {}".format(process)
    # os.system(command)
    subprocess.run('python {}'.format(process), shell=True, check=True)
    # si = subprocess.STARTUPINFO()
    # si.dwFlags = subprocess.CREATE_NEW_CONSOLE \
    #              | subprocess.STARTF_USESHOWWINDOW
    # si.wShowWindow = subprocess.SW_HIDE
    # p = subprocess.Popen(
    #     command,
    #     bufsize=1,
    #     creationflags=subprocess.CREATE_NEW_CONSOLE,
    #     cwd=cwd,
    #     shell=True,
    # )

    # subprocess.CREATE_NEW_CONSOLE


pool = Pool(processes=len(processes),)
pool.map(run_process, processes)

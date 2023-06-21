#!/usr/bin/python3 

import time 
import subprocess 
import os 
import time

# check whether px4 is already built 
def check_build(directory_path): 
	if not os.path.exists(directory_path+'/build'):    
		output = subprocess.run(['make', 'px4_sitl'], capture_output=True, text=True, cwd=px4_directory) 
		print(output.stdout) 

# open terminator 
def open_terminator(command): 
	subprocess.Popen(['terminator', '--new-tab', '-e', command]) 

# main function 
if __name__ == '__main__': 
	# px4 build check 
	px4_directory = '/home/docker/px4' 
	check_build(px4_directory) 

	# Open Terminator shells and run the commands 
	run_commands = [
		'cd ~/px4; PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=${PX4_GZ_MODEL_POSE} PX4_GZ_MODEL=${PX4_GZ_MODEL} ./build/px4_sitl_default/bin/px4',
		'MicroXRCEAgent udp4 -p 8888']

	for idx, command in enumerate(run_commands): 
		if idx==1: 
			time.sleep(5.0) 
			open_terminator(command) 
		else: 
			time.sleep(1.5) 
			open_terminator(command)
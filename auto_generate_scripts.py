#!/usr/bin/python3

import sys
import yaml

def generate_compose(base_yaml_file,num_drones):

    # parse the base yaml file
    with open(base_yaml_file) as base_file:
        base = yaml.load(base_file,Loader=yaml.FullLoader)

    # export environmental variables for the drones and add environmental variables for multiple drones
    for i in range(num_drones):
        if i == 0:
            base['services']['drones']['environment'][5]='INSTANCE_SIGN_'+str(i+1)+'='+str(i+1)
            base['services']['drones']['environment'][6]='PX4_GZ_MODEL_'+str(i+1)+'=x500'
            base['services']['drones']['environment'][7]='PX4_GZ_MODEL_POSE_'+str(i+1)+'=0,'+str(2*i)+',0,0,0,0'
            base['services']['drones']['environment'][8]='PX4_MICRODDS_NS_'+str(i+1)+'=px4_'+str(i+1)
            base['services']['drones']['environment'][9]='ROS_DOMAIN_ID_'+str(i+1)+'=px4_'+str(i+1)

        else:
            base['services']['drones']['environment'].insert(5+5*i,'INSTANCE_SIGN_'+str(i+1)+'='+str(i+1))
            base['services']['drones']['environment'].insert(6+5*i,'PX4_GZ_MODEL_'+str(i+1)+'=x500')
            base['services']['drones']['environment'].insert(7+5*i,'PX4_GZ_MODEL_POSE_'+str(i+1)+'=0,'+str(2*i)+',0,0,0,0')
            base['services']['drones']['environment'].insert(8+5*i,'PX4_MICRODDS_NS_'+str(i+1)+'=px4_'+str(i+1))
            base['services']['drones']['environment'].insert(9+5*i,'ROS_DOMAIN_ID_'+str(i+1)+'=px4_'+str(i+1))
            
    # generate the Docker Compose YAML content
    yaml_content = base
   
    return yaml_content

def generate_autorun_script(num_drones):

    # initialize shell script
    script_content = f"#!/usr/bin/python3 \n\nimport time \nimport subprocess \nimport os \nimport time\n\n"

    # build px4 if necessary
    script_content += f"# check whether px4 is already built \ndef check_build(directory_path): \n\tif not os.path.exists(directory_path+'/build'):\
    \n\t\toutput = subprocess.run(['make', 'px4_sitl'], capture_output=True, text=True, cwd=px4_directory) \n\t\tprint(output.stdout) \n\n"

    # run terminator for the number of drones
    script_content += f"# open terminator \ndef open_terminator(command): \n\tsubprocess.Popen(['terminator', '--new-tab', '-e', command]) \n\n"

    # main function
    script_content += f"# main function \nif __name__ == '__main__': \n\t# px4 build check \n\tpx4_directory = '/home/docker/px4' \n\tcheck_build(px4_directory) \n"
    script_content += f"\t# Open Terminator shells and run the commands \n\trun_commands = [\n"

    for i in range(num_drones):
        if i == num_drones-1:
            script_content += "\t\t'cd ~/px4; PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=${PX4_GZ_MODEL_POSE_"+str(i+1)+"} PX4_GZ_MODEL=${PX4_GZ_MODEL_"+str(i+1)+"} ./build/px4_sitl_default/bin/px4 -i ${INSTANCE_SIGN_"+str(i+1)+"}']\n"

        else:
            script_content += "\t\t'cd ~/px4; PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=${PX4_GZ_MODEL_POSE_"+str(i+1)+"} PX4_GZ_MODEL=${PX4_GZ_MODEL_"+str(i+1)+"} ./build/px4_sitl_default/bin/px4 -i ${INSTANCE_SIGN_"+str(i+1)+"},',\n"

    script_content += "\tfor command in run_commands: \n\t\topen_terminator(command) \n\t\ttime.sleep(1.5)"

    return script_content

if __name__ == '__main__':
    # Get the command-line arguments
    if len(sys.argv) != 2:
        print('Usage: python generate_compose.py <num_drones>')
        sys.exit(1)
    
    num_drones = int(sys.argv[1])
    base_yaml_file = 'compose_base.yaml'
   
    int_yaml_content = generate_compose(base_yaml_file,num_drones)

    with open('compose.yaml', 'w') as f:
        yaml.dump(int_yaml_content,f,sort_keys=False)

    multiple_run_python_content = generate_autorun_script(num_drones)

    with open('./dronesim/bin/px4_multirun.py','w') as f:
        f.write(multiple_run_python_content)
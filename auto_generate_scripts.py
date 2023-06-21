#!/usr/bin/python3

# load public libraries
import sys
import yaml
import argparse
import numpy

# define functions
def generate_compose(base_yaml_file,num_drones,identifier):

    # parse the base yaml file
    with open(base_yaml_file) as base_file:
        base = yaml.load(base_file,Loader=yaml.FullLoader)

    if identifier is not None:
        base['services']['drones_'+identifier]  =   base['services']['drones']
        del base['services']['drones']

        base['services']['drones_'+identifier]['container_name']=base['services']['drones_'+identifier]['container_name']+'_'+identifier
        base['services']['drones_'+identifier]['hostname']=base['services']['drones_'+identifier]['hostname']+'_'+identifier
        base['services']['drones_'+identifier]['image']=base['services']['drones_'+identifier]['image'][0:-7]+'_'+identifier+base['services']['drones_'+identifier]['image'][-7:]

        # export environmental variables for the drones and add environmental variables for multiple drones
        if num_drones == 1:

            del base['services']['drones_'+identifier]['environment'][9]
            del base['services']['drones_'+identifier]['environment'][8]
            base['services']['drones_'+identifier]['environment'][7]='PX4_GZ_MODEL_POSE=0,0,0,0,0,0'
            base['services']['drones_'+identifier]['environment'][6]='PX4_GZ_MODEL=x500'
            del base['services']['drones_'+identifier]['environment'][5]
        
        else:
            for i in range(num_drones):
                if i == 0:
                    base['services']['drones_'+identifier]['environment'][5]='INSTANCE_SIGN_'+str(i+1)+'='+str(i+1)
                    base['services']['drones_'+identifier]['environment'][6]='PX4_GZ_MODEL_'+str(i+1)+'=x500'
                    base['services']['drones_'+identifier]['environment'][7]='PX4_GZ_MODEL_POSE_'+str(i+1)+'=0,'+str(2*i)+',0,0,0,0'
                    base['services']['drones_'+identifier]['environment'][8]='PX4_MICRODDS_NS_'+str(i+1)+'=px4_'+str(i+1)
                    base['services']['drones_'+identifier]['environment'][9]='ROS_DOMAIN_ID_'+str(i+1)+'=px4_'+str(i+1)

                else:
                    base['services']['drones_'+identifier]['environment'].insert(5+5*i,'INSTANCE_SIGN_'+str(i+1)+'='+str(i+1))
                    base['services']['drones_'+identifier]['environment'].insert(6+5*i,'PX4_GZ_MODEL_'+str(i+1)+'=x500')
                    base['services']['drones_'+identifier]['environment'].insert(7+5*i,'PX4_GZ_MODEL_POSE_'+str(i+1)+'=0,'+str(2*i)+',0,0,0,0')
                    base['services']['drones_'+identifier]['environment'].insert(8+5*i,'PX4_MICRODDS_NS_'+str(i+1)+'=px4_'+str(i+1))
                    base['services']['drones_'+identifier]['environment'].insert(9+5*i,'ROS_DOMAIN_ID_'+str(i+1)+'=px4_'+str(i+1))

    else:
        if num_drones == 1:

            del base['services']['drones']['environment'][9]
            del base['services']['drones']['environment'][8]
            base['services']['drones']['environment'][7]='PX4_GZ_MODEL_POSE=0,0,0,0,0,0'
            base['services']['drones']['environment'][6]='PX4_GZ_MODEL=x500'
            del base['services']['drones']['environment'][5]
        
        else:
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
    script_content += f"# main function \nif __name__ == '__main__': \n\t# px4 build check \n\tpx4_directory = '/home/docker/px4' \n\tcheck_build(px4_directory) \n\n"
    script_content += f"\t# Open Terminator shells and run the commands \n\trun_commands = [\n"

    if num_drones == 1:
         script_content += "\t\t'cd ~/px4; PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=${PX4_GZ_MODEL_POSE} PX4_GZ_MODEL=${PX4_GZ_MODEL} ./build/px4_sitl_default/bin/px4',\n"
         
    else:
        for i in range(num_drones):
            script_content += "\t\t'cd ~/px4; PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=${PX4_GZ_MODEL_POSE_"+str(i+1)+"} PX4_GZ_MODEL=${PX4_GZ_MODEL_"+str(i+1)+"} ./build/px4_sitl_default/bin/px4 -i ${INSTANCE_SIGN_"+str(i+1)+"}',\n"

    script_content += "\t\t'MicroXRCEAgent udp4 -p 8888']\n\n"

    script_content += "\tfor idx, command in enumerate(run_commands): \n\t\tif idx=="+str(num_drones)+": \n\t\t\ttime.sleep(5.0) \n\t\t\topen_terminator(command) \n\t\telse: \n\t\t\ttime.sleep(1.5) \n\t\t\topen_terminator(command)"

    return script_content

def none_or_string(value):
    if value == 'None':
        return None
    
    return value

# main
if __name__ == '__main__':
    # Get the command-line arguments and parse
    parser  =   argparse.ArgumentParser(description='Parameters for simulations and running scripts')
    parser.add_argument('--num-drones','-n',type=int,default=numpy.uint8(1),help='number of drones')
    parser.add_argument('--identifier','-i',type=none_or_string,default=None,help='set the identifier to run in a server')
    arg_in  =   parser.parse_args()

    base_yaml_file = 'compose_base.yaml'
   
    int_yaml_content = generate_compose(base_yaml_file,arg_in.num_drones,arg_in.identifier)

    with open('compose.yaml', 'w') as f:
        yaml.dump(int_yaml_content,f,sort_keys=False)

    multiple_run_python_content = generate_autorun_script(arg_in.num_drones)

    with open('./dronesim/bin/px4_multirun.py','w') as f:
        f.write(multiple_run_python_content)
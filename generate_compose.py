import sys
import yaml

def generate_compose(base_yaml_file, num_drones):

    # Parse the base yaml file
    with open(base_yaml_file) as f:
        base = yaml.load(f,Loader=yaml.FullLoader)

    # Export environmental variables for the drones and add environmental variables for multiple drones
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
            
    # Generate the Docker Compose YAML content
    yaml_content = base
   
    return yaml_content


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
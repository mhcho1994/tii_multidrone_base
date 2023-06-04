import os
import subprocess
import grp

# check whether the autopilot source code is included in the drone/px4 folder
if not os.path.isdir("./dronesim/drone/px4"):
    os.makedirs("./dronesim/drone/px4")
    subprocess.run(["git", "clone", "https://github.com/PX4/PX4-Autopilot.git", "./dronesim/drone/px4"])
    subprocess.run(["git", "checkout", "tags/v1.14.0-beta2"], cwd="./dronesim/drone/px4")

# check whether the px4_msgs source code is included in the drone/onboard folder
if not os.path.isdir("./dronesim/drone/onboard/src/px4_msgs"):
    os.makedirs("./dronesim/drone/onboard/src/px4_msgs")
    subprocess.run(["git", "clone", "https://github.com/PX4/px4_msgs.git", "./dronesim/drone/onboard/src/px4_msgs"])

# enable the communication between containers and X windows in the host
subprocess.run(["xhost", "+local:docker"])

# set the current user name/group/uid/gid as environment variables
container_user_id = os.getuid()
container_user_name = os.getlogin()
container_group_id = os.getgid()
container_group_name = grp.getgrgid(container_group_id).gr_name

os.environ["CONTAINER_USER_NAME"] = container_user_name
os.environ["CONTAINER_USER_ID"] = str(container_user_id)
os.environ["CONTAINER_GROUP_NAME"] = container_group_name
os.environ["CONTAINER_GROUP_ID"] = str(container_group_id)
# owr-502
This repo includes all software necessary to operate the Test Rover Platform that is being designed and developed in the 502 project.

## Starting the dev environment
Run `podman compose -f "$PWD/.devcontainer/docker-compose.yml" up -d` to build and run the container, or restart if it was stopped.

Then run `podman exec -it --latest /bin/bash`

To kill the container run `podman rm --force --latest -t 0` 


### GUI Applications
#### Linux
Requires an X server (i.e XWayland).
#### OSX
Install [XQuartz](https://www.xquartz.org/).
In XQuartz settings enable network connections and disable network authentication (`DISPLAY` in the container will be `host.docker.internal:XDISPLAY_HERE` where `XDISPLAY_HERE` is not `0` (from `/tmp/.X11-unix`).
Enable indirect OpenGL rendering (IGLX) `defaults write org.xquartz.X11 enable_iglx -bool YES`

Caveats: 
RViz is not supported (Rqt will work) as it requires OpenGL 1.5 (Up to 1.4 is supported by IGLX, LLVMPipe/Softpipe did not work during testing).

#### Windows
1. Install the extension `Dev Containers` in VSCode
2. Run `wsl --install -d Ubuntu` to install WSL
3. Open a new WSL terminal
4. If prompted for a password, create a password for your Linux user account
5. Navigate to `\\wsl.localhost\Ubuntu\home\[user]`, clone and open the repository 
6. If a popup appears prompting you to reopen in a development container, click "Reopen in Container.". Similarly, if a popup appears prompting you to install a docker press `install`.
7. Press `show log` and if this is your first time building the code, wait approx 15 mins for the code to build.
8. Run `./init.bash`
9. Run `colcon build --symlink-install`
10. Run `source install/setup.bash`
11. Run `ros2 launch rover rover.launch.py`

### Post-setup
Run `./init.bash` in each new terminal. This will source the appropriate files as well as put you in a 10-deep nested shell. This is needed because running an invalid command will kick you out of your shell session.

### Command index
| Command                         | Effect                           |
| ------------------------------- | -------------------------------- |
| `devcontainer up --workspace_directory .` | Run in owr-502/ to run the Docker container. Make sure to first replace `docker-compose.yml` with `docker-compose-jetson.yml` in `devcontainer.json` before running if you are on the Jetson. |
| `docker exec -it ros2_docker /bin/bash` | Enter the docker container after launching it with the devcontainer. |
| `ros2 launch rover rover.launch.py` | Launch the robot state publisher. Just leave this running. |
| `xacro ./src/rover/description/robot.urdf.xacro > ./build/rover.urdf` | Compiles the xacro into a singular urdf file|
| `colcon build --symlink-install` | Builds the rover for the state publisher |
| `gz sim` | Opens Gazebo |
| `rviz2`  | Opens Rviz2 |
| `rqt`    | Launch rqt  |
| `gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "./build/rover.urdf", name: "rover"'` | Spawns the rover into Gazebo |
| `ros2 run key_teleop key_teleop --ros-args -p twist_stamped_enabled:=True -r /key_vel:=/diff_drive_base_controller/cmd_vel` | Allows rover movement |


### FAQ
- #### `Authorization required, but no authorization protocol specified`
  - Run `xhost +` on the host to allow X clients to **connect from anywhere** (`xhost -` to re-enable authentication)
- Can't edit any files in the container/on the host
  - reason: With podman the UID/GID are translated in/out of the container, this doesn't apply to bind mounts
  - solution: use the vscode remote dev container extension or similar (as described above) to only edit files from in the container
- Still can't edit any files in the container/on the host
  - reason: Docker/Podman set them to belong to the user "Ubuntu" for some reason
  - solution: run `sudo chown -R dcuser .` . Note that this will block you from editing files from outside the container, in which case you will need to run `sudo chown -R <your_user_name> .` .
- Where is the code?
  - `/workspaces/owr-502` in the container


# Jetson Configuration
## Apt Packages Needed

``` shell
sudo apt install docker docker-compose docker-buildx git
```

You will also need to run:

``` shell
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker # Or preferably, log out and log in again.
```

## NVM
Nodejs is needed for the devcontainer package.
For some reason the one provided by apt on the Jetson image is years out of date.
Solution: [Download NVM using the install script](https://github.com/nvm-sh/nvm).
The Jetson comes with wget pre-installed, but not curl.

Then run:

``` shell
sudo npm install -g @devcontainers/cli
```

## Git
Follow the instructions for ssh setup [here](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent).
Then run `git config --global user.name <>, git config --global user.email <>`.
You can use your own github email, but by convention make the username some variation of "Wark".

### List of Existing Warks
Avoid naming any new Jetson git users any of these names:

- Wark Maldron
- Wark 2

Add to this list as more Warks are brought into existence.

# owr-502
This repo includes all software necessary to operate the Test Rover Platform that is being designed and developed in the 502 project.

## Starting the dev environment
Run `devcontainer up --workspace-folder .` to build and run the dev container, or restart if it was stopped.

Caveats:
- Anything capable of launching a devcontainer-compliant container will work (code-oss and vscodium will **not** work)
  - As a fallback the [devcontainer cli](https://github.com/devcontainers/cli) may be used
  - Alternatively use [vscode-remote-oss](https://github.com/xaberus/vscode-remote-oss) or [vscode-open-remote-ssh](https://open-vsx.org/extension/jeanp413/open-remote-ssh) (more basic)
- The devcontainer CLI cannot stop or remove containers

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
Set up [WSLg](https://github.com/microsoft/wslg/blob/main/samples/container/Containers.md).
Install docker inside WSL

### FAQ
- #### `Authorization required, but no authorization protocol specified`
  - Run `xhost +` on the host to allow X clients to **connect from anywhere** (`xhost -` to re-enable authentication)
- Can't edit any files in the container/on the host
  - reason: With podman the UID/GID are translated in/out of the container, this doesn't apply to bind mounts
  - solution: use the vscode remote dev container extension or similar (as described above) to only edit files from in the container
- Where is the code?
  - `/workspaces/owr-502` in the container

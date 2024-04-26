
# Getting started
This file has information to get you started developing the robot.

## 1 Checkout the code 
*If you have used git and GitHub before, follow this quick start guide. If not, navigate to [github.md](https://github.com/CabrilloRoboticsClub/cabrillo_rov_2023/blob/main/doc/github.md).*

**These instructions are for Mac, Windows and Linux.** On Windows you should use PowerShell, not the old command line. To get the code in this repository on your computer, dev box or RPi you should run the following commands. These commands assume that you have setup your SSH keys in GitHub.

```sh
cd 
git clone git@github.com:CabrilloRoboticsClub/cabrillo_rov_2023.git
cd ~/cabrillo_rov_2023
git submodule init 
git submodule update
```

Now you have the source code on your computer and you should be able to make new branches and your own changes. 


## 2 Mac and windows setup
If you have a Windows or Mac desktop (or Linux that's not Ubuntu 22.04) you can use Docker and Development Containers in vscode to do local development. This is the fastest and easiest way to get setup and enables you to run and test nodes on your computer. That's good for learning but you won't be able to use the ROS2 network or use devices like a joystick or camera.

1. [Install vscode](https://code.visualstudio.com/)
1. [Install Docker Desktop](https://www.docker.com/products/docker-desktop/)
1. Follow the [Dev Containers Tutorial](https://code.visualstudio.com/docs/devcontainers/tutorial)
1. In vscode open the `cabrillo_rov_2023` directory that you checked out using the "1 Checkout the code" procedure above with the `File -> Open Folder` menu. Vscode will prompt you to create a dev container. Follow the prompts. Building the container could take 10 to 20 minutes but you only have to do it once. 
1. Create a Terminal in your vscode window with the `Terminal -> New Terminal` menu. 
1. Build the software in your dev container: 

    **Run this command in the dev container terminal.**

    ```sh 
    make 
    ```

Alternatively, if you don't want to use the container inside of VSCode, you can run
```sh
docker build -t ros2-container:<tag> .devcontainer
docker run --user vscode -v .:/home/vscode/cabrillo_rov_2023 --network host -it ros2-container:<tag>
```

## 2 Ubuntu 22.04 Desktop Setup 

If you have Ubuntu 22.04 on your desktop you don't need a dev container because you can run everything natively. Only Ubuntu machines will be able to use hardware and communicate with other machines in the ROS2 network. These instructions assume you have already cloned the repository by following the "1 Checkout the code" procedure above.

1. Install the necessary software packages
    ```sh
    sudo apt update -y && sudo apt install -y git ansible python3-pip vim 
    ```
1. Use `make` to install ROS2 and all the necessary packages. **This will take a while!** 
    ```sh 
    make devbox-install
    ```

1. Put ROS on your path 
    ```sh
    echo 'if [ -z "$ROS_DISTRO" ]; then source /opt/ros/humble/setup.bash; fi' >> ~/.bashrc 
    ```
1. **Start a new shell** and build the repository
    ```sh
    make 
    ```

You Desktop is now a dev box. 

## 2 Raspberry Pi 4 Setup

### 2.1 Start by creating a bootable SD card image

1. On your local computer download and install [Raspberry Pi Imager](https://www.raspberrypi.com/software/)
1. Attach a Micro SD card to your local computer
1. Open Raspberry Pi Imager
    * `CHOOSE OS` > `Other general-purpose OS` > `Ubuntu` > `Ubuntu Server 22.04 LTS (64-bit)`
    * Choose the Micro SD card as the storage and hit `WRITE`
    ![Pi imager options](/doc/img/pi_imager.png)
    * Do not apply any OS customization settings
1. Unplug the Micro SD card from your local computer, plug it back in to your local computer.
1. Copy the [`user-data`](https://github.com/CabrilloRoboticsClub/cabrillo_rov_2023/blob/main/setup/image/user-data) file from the `setup/image` directory on this repo into the `system-boot` volume that shows up (overwrite the existing file).
1. Install the SD card into the Raspberry Pi
1. Connect the Pi to an ethernet connection with internet access
1. Plug the power cable in to the Pi.
The Pi will boot, then reboot. Give it 10 minutes or so. The hostname and passwords are now set. SSH is enabled. You can connect with:
    ```sh
    ssh -A ubuntu@seahawk.local
    ```

### 2.2 Once you have SSHed into the new Pi image run the ROV specific playbooks

1. Check out this repository into your home directory. 
    ```sh
    cd 
    git clone https://github.com/CabrilloRoboticsClub/cabrillo_rov_2023.git
    cd ~/cabrillo_rov_2023
    git submodule init 
    git submodule update
    ```
1. Install make and Ansible command
    ```sh
    sudo apt install make ansible
    ```
1. Use `make` to install ROS2 and all the necessary packages. 
    ```sh 
    make rov-install
    ```
    **This could take a long, long time!** up to 30 minutes. 

1. Reboot the Pi 
    ```sh
    sudo reboot
    ```

The `make rov-install` command installs ROS2 and creates user accounts on the Raspberry Pi. After rebooting you should be able to login with your own username. 

### 2.3 Using vscode to Connect to a Pi

The best way to access the Raspberry Pi for software development is using vscode. You can use the access to develop and test nodes that run on the Pi or to just start the ROV during development of deck nodes. Assuming you have your own computer setup per the instructions above, perform the following steps to access the Pi. 

1. Install the [Remote SSH plugin](https://code.visualstudio.com/docs/remote/remote-overview) into vscode
1. Click the remote status bar on the very bottom left of the window. (You can find some tips in the [Remote SSH tutorial](https://code.visualstudio.com/docs/remote/ssh-tutorial).)

    ![](https://code.visualstudio.com/assets/docs/remote/ssh-tutorial/remote-status-bar.png)

    The Remote Status bar item can quickly show you in which context VS Code is running (local or remote) and clicking on the item will bring up the Remote - SSH commands.

    ![](https://code.visualstudio.com/assets/docs/remote/ssh-tutorial/remote-ssh-commands.png)
1. Type the ssh command into the top bar: 

    ![](https://code.visualstudio.com/assets/docs/remote/ssh-tutorial/set-user-host.png)

    > The command will look like: **ssh yourname@seachicken.local -A** 

1. Follow the prompts:
    1. Choose the first option for where to save the configuration. 
    1. Choose "Linux" type if asked. 
    1. Choose "Yes" if this is your first time connecting to this Pi.

1. You should see that you are connected:

    ![](https://code.visualstudio.com/assets/docs/remote/ssh-tutorial/ssh-status-bar.png)

1. Now that you're connected, start a new Terminal using the `Terminal -> New Terminal` menu. 
1. Create a copy of the repo in your home directory: 

    ```sh 
    cd
    git clone git@github.com:CabrilloRoboticsClub/cabrillo_rov_2023.git
    cd ~/cabrillo_rov_2023
    git submodule init 
    git submodule update
    ```

1. Configure git on the Pi4:

    ```sh 
    git config --global user.name "Your Name"
    git config --global user.email "you@you.com"
    ```

1. Add ROS to your startup scripts:

    ```sh 
    echo 'if [ -z "$ROS_DISTRO" ]; then source /opt/ros/humble/setup.bash; fi' >> ~/.bashrc 
    ```

1. Exit your current terminal and start a new one. 
1. Build the ROS project

    ```sh 
    cd ~/cabrillo_rov_2023
    make
    ```

Vscode will remember everything for next time. You should only have to do this once. 

## 3 Next steps
Navigate to [`run.md`](https://github.com/CabrilloRoboticsClub/cabrillo_rov_2023/blob/main/doc/run.md) for information on how to run the project.
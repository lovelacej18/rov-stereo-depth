#rfal

##### Specifications
GRAPHICS -> NV137/GeForce 1060, 1070, Pascal
OS -> Ubuntu 20.04

##### Installations
**gcc**
```
sudo apt update
sudo apt install build-essential
gcc --version
sudo apt install cmake
```

**NVIDIA Driver**
Running 525 - going through the Ubuntu software and updates graphical menu

**CUDA**
Installing 11.7
```
wget https://developer.download.nvidia.com/compute/cuda/11.7.0/local_installers/cuda_11.7.0_515.43.04_linux.run
sudo sh cuda_11.7.0_515.43.04_linux.run
```

Updating PATH and LD_LIBRARY_PATH with new info
```
export PATH=/usr/local/cuda-11.7/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-11.7/lib64\{LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
```

**git**
```
sudo apt install git
git --version
git config --global user.email "youremail@yourdomain.com"
git config --global user.name "username"
git config --list
```
https://docs.github.com/en/authentication/connecting-to-github-with-ssh/checking-for-existing-ssh-keys
Enter ```ls -al ~/.ssh```  to see if existing SSH keys are present.

**LFG (large file git)**
For debian (we are running ubuntu so I'm pretty sure its fine...)
```
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
sudo apt-get install git-lfs
```

**Python3** -> 

**ROS Noetic**
ROS1
Already installed but:
```
echo "deb http://packages.ros.org/ros/ubuntu focal main" | sudo tee /etc/apt/sources.list.d/ros-focal.list
```
```
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

**Terminator**
```
sudo apt-get install terminator
```

**ZED SDK**
Downloading 3.8.2 for CUDA 11.7, Ubuntu 20
Go to the folder where the installer has been downloaded
```
sudo apt install zstd
```
Make the downloaded file executable... and then execute it! 
```
chmod +x ZED_SDK_Ubuntu20_cuda11.7_v3.8.2.zstd.run
./ZED_SDK_Ubuntu20_cuda11.7_v3.8.2.zstd.run
```

**CudNN**



##### Pre-Existing on Computer
Ubuntu 20.04
VSCode
ROS Noetic (newest version!)

---

Almost re-installed ubuntu when the network wasn't showing up - but now we're good! 


---

*Update 05-30-2023*
Installing **`zed-ros-wrapper`** v3.8.x
Needed to download zip of above repo and **`zed-ros-interfaces`**, migrate the latter into the earlier

Installing **`ros_bridge`**, [using source repository][https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros1/]

Installing **`ros-noetic-tf2-geometry`**

--- 
Another install!
```
sudo apt install ros-noetic-ros-base
```

cmake clean and rebuilding fixed it!


**MATLAB**

Downloaded from Stevens digital backpack and the subsequent wizard that popped up. Follow Stevens-specific instruction, as they're the ones who have the MATLAB academic liscence, not I.

[Stevens Student Digital Backpack](https://www.stevens.edu/tools/student-digital-backpack)
[Software Download Store - must signin with Stevens email](https://software.stevens.edu/)

**png++**
Download from [Savannah](https://savannah.nongnu.org/projects/pngpp/)

[png++ docs](https://www.nongnu.org/pngpp/doc/0.2.9/) - includes install information. Be mindful to unpack the version you downloaded.

```
tar -zxf png++-0.2.x.tar.gz -C ~/src
cd ~/src/png++-0.2.x
make
make test
```

Everything from this point is optional, but its a good idea. 

```
make docs
make install PREFIX=$HOME
```
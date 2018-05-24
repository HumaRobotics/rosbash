#!/bin/bash

# ROS NETWORK CONFIGURATION

rosnetwork() { 
    env | egrep "ROS_MASTER_URI|ROS_IP|ROS_HOSTNAME" 
}

rosmaster() {    
    export ROS_MASTER_URI=http://$1:11311
    rosnetwork
    rosprompt
}

rosip() {    
    export ROS_IP=$1
    unset ROS_HOSTNAME
    rosnetwork
}

roshostname() {    
    export ROS_HOSTNAME=$1
    unset ROS_IP
    rosnetwork
}

# PROMPT AND SHELL

rosprompt() {
    MASTER=`echo $ROS_MASTER_URI  | sed 's/http:\/\///' | sed 's/:11311//'`

    # Extract top folder last componentent
    ROSPATHNAME=`(roscd;cd ..; pwd | sed -e "s/.*\///g"  )`
    export PS1='\[\033[0;31m\]${ROS_DISTRO[@]:0:1} \[\033[0;34m\]$ROSPATHNAME\[\033[0;32m\]@$MASTER\[\033[0m\]:\[\033[0;36m\]\w\[\033[0m\]> '
}

# Loads child Bash environment with bashrc, prompt, and any start command as a parameter
rosshell() {
    F=`mktemp`    
    echo source ~/.bashrc >> $F
    echo $* >> $F
    echo rosprompt >> $F
    #~ echo ros >> $F
    bash --rcfile $F    
}

urdf_display() {
    roslaunch urdf_tutorial display.launch gui:=True model:=$1
}

xacro_display() {
    rosrun xacro xacro "$1" >"$1.urdf"
    roslaunch urdf_tutorial display.launch gui:=True model:="$1.urdf"
}

# ENVIRONMENT AND SOURCING

alias ros=' env | egrep "ROS_.*=|PYTHONPATH|LD_LIBRARY" '
alias indigo='rosshell source /opt/ros/indigo/setup.bash'
alias hydro='rosshell source /opt/ros/hydro/setup.bash'
alias groovy='rosshell source /opt/ros/groovy/setup.bash'
alias kinetic='rosshell source /opt/ros/kinetic/setup.bash'
alias devel='rosshell source devel/setup.bash'
alias install='rosshell source install/setup.bash'
alias install_deps="(roscd;cd ..;rosdep install --from-paths src --ignore-src --rosdistro hydro)"

alias rosrefresh='(roscd;cd ..; rospack profile)'
alias pydev='python $(rospack find mk)/make_pydev_project.py'

alias rosdep_indigo='rosdep install -r --from-paths src --ignore-src --rosdistro indigo -y'

# SHORTHAND
alias cm='(roscd && cd ..; catkin_make)'
alias rn='rosnode list'
alias rni='rosnode info'
alias rte='rostopic echo'
alias rtl='rostopic list'
alias rti='rostopic info'

# Generates debian package from ROS package name
todeb() {
    # Get OS name and codename
    OS="$(lsb_release -c | cut -f2)"
    NAME="$(lsb_release -i | cut -f2 | tr "[:upper:]" "[:lower:]")"
    # Remember current dir
    local ORIG_DIR="$(pwd)"
    # Create deb dir if necessary and store path
    DEB_DIR="deb"
    roscd && cd ..
    if [ ! -d "$DEB_DIR" ]; then
        mkdir $DEB_DIR
    fi
    DEB_DIR="$(cd $DEB_DIR && pwd)"
    # Generate debian package
    roscd $1 &&
    # Remove old debian and obj-* dirs
    rm -rf debian obj-*
    # Inject private package key resolutions into rosdep
    inject-rosdeps &&
    # Generate and build .deb
    bloom-generate rosdebian --os-name $NAME --os-version $OS --ros-distro $ROS_DISTRO &&
    fakeroot debian/rules binary &&
    # Stores deb package in deb directory.
    mv ../*.deb $DEB_DIR
    # Remove rosdep injections
    withdraw-rosdeps
    # Remove new debian and obj-* dirs
    rm -rf debian obj-*
    # Return to initial dir for convenience
    cd $ORIG_DIR
}

# Inject dependency key resolutions in rosdep for private packages in a catkin workspace
inject-rosdeps() {
    # Remember current dir
    local ORIG_DIR="$(pwd)"
    # Remove any old injections
    withdraw-rosdeps
    # Get OS name and codename
    OS="$(lsb_release -c | cut -f2)"
    NAME="$(lsb_release -i | cut -f2 | tr "[:upper:]" "[:lower:]")"
    # Get packages
    roscd
    cd ..
    PACKS=$(catkin list --quiet -u)
    # Go to rosdep sources location
    cd /etc/ros/rosdep/sources.list.d/
    # Create list file
    sudo touch 50-injections.list
    echo "yaml file:///etc/ros/rosdep/sources.list.d/injected-keys.yaml" | sudo tee -a 50-injections.list
    # Create key resolution yaml
    sudo touch injected-keys.yaml
    for pack in $PACKS; do
        key="ros-${ROS_DISTRO}-$(echo $pack | sed 's/_/-/g')"
        echo -e "${pack}:\n  $NAME:\n    $OS: [$key]" | sudo tee -a injected-keys.yaml
    done
    # Return to initial dir for convenience
    cd $ORIG_DIR
}

# Remove injected dependencies from rosdep
withdraw-rosdeps() {
    # Remember current dir
    local ORIG_DIR="$(pwd)"
    # Go to rosdep sources location
    cd /etc/ros/rosdep/sources.list.d/
    # Remote injected files
    sudo rm -f 50-injections.list injected-keys.yaml
    # Return to initial dir for convenience
    cd $ORIG_DIR
}

install_todeb() {
    ## Install dependencies for todeb
    sudo apt update &&
    sudo apt install python-bloom dpkg-dev debhelper -y
}

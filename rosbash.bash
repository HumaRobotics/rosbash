#!/bin/bash

# ROS NETWORK CONFIGURATION

rosnetwork() {
    env | egrep "ROS_MASTER_URI|ROS_IP|ROS_HOSTNAME"
}

rosmaster() {
    if [ $# -eq 0 ]
    then
        echo "Setting localhost"
        export ROS_MASTER_URI=http://localhost:11311
        unset ROS_IP
        unset ROS_HOSTNAME
    else
        export ROS_MASTER_URI=http://$1:11311
    fi
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

parse_git_branch() {
    git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/(\1)/'
}

rosprompt() {
    MASTER=`echo $ROS_MASTER_URI  | sed 's/http:\/\///' | sed 's/:11311//'`
    # Extract top folder last componentent
    ROSPATHNAME=`(roscd;cd ..; pwd | sed -e "s/.*\///g"  )`

    export PS1='\[\033[0;31m\]${ROS_DISTRO[@]:0:1} \[\033[0;34m\]$ROSPATHNAME\[\033[33m\]$(parse_git_branch)\[\033[0;32m\]@$MASTER\[\033[0m\]:\[\033[0;36m\]\w\[\033[0m\]\[\033[00m\]> '
}

toggle-hostname() {
    local PATTERN='^\$\(hostname\) .+$'
    if [[ $PS1 =~ $PATTERN ]]; then
        export PS1=${PS1#\$(hostname) }
    else
        export PS1="\$(hostname) $PS1"
    fi
}

# Loads child Bash environment with bashrc, prompt, and any start command as a parameter
rosshell() {
    F=`mktemp`
    echo source ~/.bashrc >> $F
    echo "$* || exit 1" >> $F
    echo rosprompt >> $F
    echo "env | grep ROS_PACKAGE_PATH" >> $F
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

alias melodic='rosshell source /opt/ros/melodic/setup.bash'
alias lunar='rosshell source /opt/ros/lunar/setup.bash'
alias kinetic='rosshell source /opt/ros/kinetic/setup.bash'
alias jade='rosshell source /opt/ros/jade/setup.bash'
alias indigo='rosshell source /opt/ros/indigo/setup.bash'
alias hydro='rosshell source /opt/ros/hydro/setup.bash'
alias groovy='rosshell source /opt/ros/groovy/setup.bash'
alias kinetic='rosshell source /opt/ros/kinetic/setup.bash'
alias lunar='rosshell source /opt/ros/lunar/setup.bash'
alias melodic='rosshell source /opt/ros/melodic/setup.bash'

alias devel='rosshell source devel/setup.bash'
alias install='rosshell source install/setup.bash'

alias rosrefresh='(roscd;cd ..; rospack profile)'
alias catkin_eclipse='(roscd;cd ..; catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles")'
alias install_deps="(roscd;cd ..;rosdep install --from-paths src --ignore-src -y)"

alias pydev='python $(rospack find mk)/make_pydev_project.py'

alias rosdep_indigo='rosdep install -r --from-paths src --ignore-src --rosdistro indigo -y'
alias rosdep_kinetic='rosdep install -r --from-paths src --ignore-src --rosdistro kinetic -y'
alias rosdep_lunar='rosdep install -r --from-paths src --ignore-src --rosdistro lunar -y'
alias rosdep_melodic='rosdep install -r --from-paths src --ignore-src --rosdistro melodic -y'

# SHORTCUTS
alias cm='(roscd && cd ..; catkin_make)'
alias rn='rosnode list'
alias rni='rosnode info'
alias rte='rostopic echo'
alias rtl='rostopic list'
alias rti='rostopic info'

alias gkill='killall gzserver ; killall gzclient ; killall rosout ; pkill -9 -f "python /opt/ros/" '
alias rkill='killall rosout ; pkill -9 -f "python /opt/ros/" ; gkill'
alias make-eclipse-project='cmake -G "Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug'

# Generates debian package from ROS package name
todeb() {
    # Get OS name and codename
    local OS_VERSION="$(lsb_release -c | cut -f2)"
    local OS_NAME="$(lsb_release -i | cut -f2 | tr "[:upper:]" "[:lower:]")"
    # Remember current dir
    local ORIG_DIR="$(pwd)"
    # Create deb dir if necessary and store path
    local DEB_DIR="deb"
    roscd && cd ..
    if [ ! -d "$DEB_DIR" ]; then
        mkdir $DEB_DIR
    fi
    local DEB_DIR="$(cd $DEB_DIR && pwd)"
    # Install public deps
    rosdep install -i $1
    # Generate debian package
    roscd $1 &&
    # Remove old debian and obj-* dirs
    rm -rf debian obj-*
    # Inject private package key resolutions into rosdep
    inject-rosdeps &&
    # Generate and build .deb
    local EXIT_STATUS=0
    bloom-generate rosdebian --os-name $OS_NAME --os-version $OS_VERSION --ros-distro $ROS_DISTRO &&
    fakeroot debian/rules binary
    if (( $? )); then
        local EXIT_STATUS=1
    else
        LAST_BUILT_PKG=$(basename ../*.deb)
        # Stores deb package in deb directory.
        mv ../*.deb $DEB_DIR
    fi
    # Remove rosdep injections
    withdraw-rosdeps
    # Remove new debian and obj-* dirs
    rm -rf debian obj-*
    # Return to initial dir for convenience
    cd $ORIG_DIR
    return $EXIT_STATUS
}

# Inject dependency key resolutions in rosdep for private packages in a catkin workspace
inject-rosdeps() {
    # Remember current dir
    local ORIG_DIR="$(pwd)"
    # Remove any old injections
    withdraw-rosdeps
    # Get OS name and codename
    local OS_VERSION="$(lsb_release -c | cut -f2)"
    local OS_NAME="$(lsb_release -i | cut -f2 | tr "[:upper:]" "[:lower:]")"
    # Get packages
    roscd
    cd ..
    local PACKS=$(catkin list --quiet -u)
    # Go to rosdep sources location
    cd /etc/ros/rosdep/sources.list.d/
    # Create list file
    sudo touch 50-injections.list
    echo "yaml file:///etc/ros/rosdep/sources.list.d/injected-keys.yaml" | sudo tee -a 50-injections.list
    # Create key resolution yaml
    sudo touch injected-keys.yaml
    for pack in $PACKS; do
        key="ros-${ROS_DISTRO}-$(echo $pack | sed 's/_/-/g')"
        echo -e "${pack}:\n  $OS_NAME:\n    $OS_VERSION: [$key]" | sudo tee -a injected-keys.yaml
    done
    rosdep update
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

install-rosbash() {
    ## Install dependencies for some rosbash functions
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
    sudo apt-get update
    sudo apt-get install python-catkin-tools python-bloom dpkg-dev debhelper -y
}

# Generate deb files for all private packages in repo; installs them
all-todeb() {
    # Remember current dir
    local ORIG_DIR="$(pwd)"
    roscd && cd ..
    rm -rf deb/
    local ALL_BUILT=0
    local SUCCESS=1
    sudo rm -f /run/built_pkgs.txt
    sudo touch /run/built_pkgs.txt
    while (( ! $ALL_BUILT )); do
        local NUM_BUILT=$(wc -l < /run/built_pkgs.txt)
        ALL_BUILT=1
        for p in $(catkin list --quiet -u | grep -Fxv -f /run/built_pkgs.txt); do
            todeb $p
            if (( $? )); then
                ALL_BUILT=0
            else
                cd deb
                sudo dpkg -i $LAST_BUILT_PKG &&
                echo $p | sudo tee -a /run/built_pkgs.txt
                cd -
            fi
        done
        if [ $NUM_BUILT -eq $(wc -l < /run/built_pkgs.txt) ]; then
            echo All packages cannot be built. Ending construction...
            SUCCESS=0
            break
        fi
    done
    if (( $SUCCESS )); then
        echo All packages built and installed successfully.
    else
        echo The following packages were not built or failed installation:
        echo $(catkin list --quiet -u | grep -Fxv -f /run/built_pkgs.txt)
    fi
    sudo rm /run/built_pkgs.txt
    # Return to initial dir for convenience
    cd $ORIG_DIR
}

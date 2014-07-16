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
    export PS1='\[\033[0;31m\]ROS \[\033[0;34m\]$ROS_DISTRO\[\033[0;32m\]@$MASTER\[\033[0m\]:\[\033[0;36m\]\w\[\033[0m\]> '
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



# ENVIRONMENT AND SOURCING

alias ros=' env | egrep "ROS_.*=|PYTHONPATH|LD_LIBRARY" '
alias hydro='rosshell source /opt/ros/hydro/setup.bash'
alias groovy='rosshell source /opt/ros/groovy/setup.bash'
alias devel='rosshell source devel/setup.bash'
alias install='rosshell source install/setup.bash'

alias cm='(roscd;cd ..; catkin_make)'
alias catkin_eclipse='catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles"'

# BAXTER SHORTCUTS

alias be='rostopic pub -1 /robot/set_super_enable std_msgs/Bool True'
alias bd='rostopic pub -1 /robot/set_super_enable std_msgs/Bool False'

# ================ Assignment Aliases ================
alias eight="roslaunch com2009_assignment2 task1.launch"
alias task1="roslaunch team1 task1.launch"

alias obstacle="roslaunch com2009_assignment2 obstacle_avoidance.launch"
alias task2="roslaunch team1 task2.launch"

alias maze="roslaunch com2009_assignment2 maze_nav.launch"
alias task3="roslaunch team1 task3.launch"

# supply parameter value a, b or c to alias, representing startzones
alias beaconing='f(){ roslaunch com2009_assignment2 beaconing.launch start_zone:="$1";  unset -f f; }; f'
alias beacon_colours="roslaunch com2009_assignment2 beacon_colours.launch"
alias task4="roslaunch team1 task4.launch"

alias exploration="roslaunch com2009_assignment2 exploration.launch"
alias task5="roslaunch team1 task5.launch target_colour:=red"

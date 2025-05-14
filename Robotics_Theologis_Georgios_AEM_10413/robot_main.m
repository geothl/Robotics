%%%% Main script for all cases%%%%%

%Three cases of CLICK control have been implemented

%Each case has its own .m script

%It is advised that each robot_casex.m is run seperately.

%Else all cases can be recreated by this robot_main.m file

%The downside is that lot of diagrams will be created

%%%%%%%%%%%%% CASE 1 = Σταθερές ταχύτητες 

robot_case1();

%%%%%%%%%%%%% CASE 2 = Ομαλές ταχύτητες και απότομη αλλαγή προσανατολισμού αναφοράς 

robot_case2();

%%%%%%%%%%%%% CASE 3 = Ομαλές ταχύτητες και ομαλή αλλαγή προσανατολισμού αναφοράς 

robot_case3();
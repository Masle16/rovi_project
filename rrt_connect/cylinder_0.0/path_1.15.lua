wc = rws.getRobWorkStudio():getWorkCell()
state = wc:getDefaultState()
device = wc:findDevice("UR-6-85-5-A")
gripper = wc:findFrame("Tool")
bottle = wc:findFrame("Cylinder")
table = wc:findFrame("Table")

function setQ(q)
qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])
device:setQ(qq,state)
rws.getRobWorkStudio():setState(state)
rw.sleep(0.1)
end

function attach(obj, tool)
rw.gripFrame(obj, tool, state)
rws.getRobWorkStudio():setState(state)
rw.sleep(0.1)
end

setQ({2.185 , -1.795 , -1.987 , -0.915 , 1.571 , 0})
attach(bottle, gripper)
setQ({2.15871 , -1.55622 , -1.24171 , -1.50472 , 1.4269 , -0.583778})
setQ({1.65842 , -1.31782 , -1.20754 , -1.013 , 1.33642 , -0.157677})
setQ({0.989519 , -0.99909 , -1.16186 , -0.355579 , 1.21545 , 0.412023})
setQ({0.318979 , -1.38047 , -1.69564 , -0.617746 , 0.940958 , -0.134258})
setQ({-0.283131 , -1.48824 , -2.09119 , -0.851835 , 1.61109 , -0.670923})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

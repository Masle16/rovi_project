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

setQ({1.693 , -1.728 , -2.068 , -0.932 , 1.571 , 0})
attach(bottle, gripper)
setQ({1.48858 , -1.54144 , -1.47646 , -1.00913 , 1.27526 , -0.450089})
setQ({1.14098 , -1.60823 , -1.36458 , -1.16249 , 1.33015 , -0.2825})
setQ({0.46743 , -1.73764 , -1.14777 , -1.45966 , 1.43652 , 0.042239})
setQ({-0.206121 , -1.86705 , -0.93097 , -1.75683 , 1.54288 , 0.366978})
setQ({-0.676862 , -1.83097 , -1.43513 , -1.29239 , 1.47132 , 0.210118})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

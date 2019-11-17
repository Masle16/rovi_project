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
setQ({2.30066 , -1.4069 , -1.83041 , -0.438593 , 1.59276 , -0.764226})
setQ({1.72666 , -1.47188 , -1.85743 , -0.520906 , 1.58914 , -0.637248})
setQ({0.76644 , -1.58059 , -1.90262 , -0.658604 , 1.58309 , -0.424832})
setQ({-0.19378 , -1.68929 , -1.94781 , -0.796302 , 1.57705 , -0.212416})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

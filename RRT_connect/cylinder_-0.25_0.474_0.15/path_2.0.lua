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

setQ({2.5 , -2.099 , -1.593 , -0.991 , 1.571 , 0})
attach(bottle, gripper)
setQ({2.69275 , -2.11751 , -0.281985 , -1.82213 , 0.954681 , 1.0831})
setQ({1.14971 , -1.67412 , -0.640289 , -1.50151 , 0.558152 , 0.803713})
setQ({-0.619103 , -1.16586 , -1.05102 , -1.13396 , 0.103604 , 0.483444})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

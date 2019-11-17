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
setQ({2.64817 , -2.02176 , -0.888104 , -2.55209 , 1.52211 , -0.915598})
setQ({2.16346 , -1.79873 , -0.968064 , -1.84031 , 1.60881 , -0.86937})
setQ({1.11154 , -1.31471 , -1.1416 , -0.295613 , 1.79696 , -0.769044})
setQ({0.0743576 , -1.92774 , -1.02482 , -0.0357921 , 1.13132 , 0.582523})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

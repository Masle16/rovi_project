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
setQ({2.74808 , -0.793659 , -0.890702 , -2.35599 , 0.0247016 , -1.15199})
setQ({2.17864 , -1.2986 , -1.10093 , -1.6263 , 0.524599 , -0.289614})
setQ({1.09111 , -2.26294 , -1.50244 , -0.232714 , 1.47931 , 1.35737})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

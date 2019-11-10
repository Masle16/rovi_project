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
setQ({2.30476 , -1.74668 , -0.709618 , -1.50564 , 1.39172 , 0.447657})
setQ({1.37262 , -1.56206 , -1.23534 , -1.60433 , 0.999332 , 0.723976})
setQ({0.433745 , -1.3761 , -1.76486 , -1.70374 , 0.604105 , 1.00229})
setQ({-0.358205 , -2.09932 , -1.30751 , -1.72865 , 0.536659 , 0.727645})
setQ({-0.597604 , -1.81767 , -1.08673 , -1.46061 , 1.42289 , 0.0972763})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

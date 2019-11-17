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
setQ({2.65552 , -1.52932 , -0.457269 , -1.26931 , 1.91711 , -0.752752})
setQ({1.6497 , -1.60026 , -0.862746 , -1.18078 , 1.82573 , -0.554004})
setQ({0.247849 , -1.69913 , -1.42787 , -1.05739 , 1.69836 , -0.277002})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

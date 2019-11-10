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
setQ({2.15878 , -1.49393 , -1.29148 , -1.72881 , 0.601363 , -0.708512})
setQ({1.7568 , -1.49092 , -1.34643 , -1.71137 , 0.552049 , -0.630561})
setQ({0.212391 , -1.47936 , -1.55755 , -1.64435 , 0.36258 , -0.331073})
setQ({-1.33202 , -1.4678 , -1.76867 , -1.57732 , 0.173112 , -0.0315852})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

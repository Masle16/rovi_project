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
setQ({1.59755 , -1.14966 , -1.84358 , -0.390645 , 0.594086 , 0.22051})
setQ({1.1904 , -1.2456 , -1.86569 , -0.471046 , 0.738642 , 0.187881})
setQ({0.0181987 , -1.5218 , -1.92935 , -0.702523 , 1.15482 , 0.0939406})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

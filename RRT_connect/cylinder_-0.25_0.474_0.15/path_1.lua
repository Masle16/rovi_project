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
setQ({2.51256 , -2.04624 , -0.981665 , -1.56613 , 1.0325 , 0.0507064})
setQ({1.66548 , -1.98889 , -1.21531 , -1.42009 , 1.15691 , 0.0389918})
setQ({0.725656 , -1.92526 , -1.47454 , -1.25806 , 1.29494 , 0.0259946})
setQ({-0.214172 , -1.86163 , -1.73377 , -1.09603 , 1.43297 , 0.0129973})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

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
setQ({2.65365 , -2.08637 , -0.480456 , -3.45154 , -0.79739 , -0.186935})
setQ({2.47224 , -2.0202 , -0.361842 , -2.92803 , -0.386626 , -0.084896})
setQ({1.55288 , -1.68485 , 0.239311 , -0.274819 , 1.69518 , 0.432253})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

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
setQ({2.51952 , -1.82994 , -1.65677 , -0.740845 , 0.966568 , 0.367753})
setQ({2.44449 , -1.8301 , -1.65595 , -0.756369 , 0.979619 , 0.357637})
setQ({1.67872 , -1.83175 , -1.6476 , -0.914829 , 1.11284 , 0.25438})
setQ({0.912947 , -1.83339 , -1.63925 , -1.07329 , 1.24606 , 0.151123})
setQ({0.147174 , -1.83504 , -1.6309 , -1.23175 , 1.37928 , 0.0478654})
setQ({-0.618599 , -1.83669 , -1.62255 , -1.39021 , 1.5125 , -0.0553918})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

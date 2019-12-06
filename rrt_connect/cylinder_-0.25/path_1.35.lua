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
setQ({1.95019 , -1.50863 , -1.26089 , -1.87731 , 1.09703 , 0.226263})
setQ({1.17876 , -1.41156 , -1.43253 , -1.97156 , 0.508256 , -0.326905})
setQ({0.26378 , -1.29643 , -1.6361 , -2.08336 , -0.190072 , -0.982999})
setQ({-0.765839 , -1.22859 , -1.49293 , -2.18456 , -0.164102 , -0.130727})
setQ({-0.815125 , -1.50198 , -2.03645 , -1.57647 , 0.620972 , 0.5503})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

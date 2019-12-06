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

setQ({2.185 , -1.795 , -1.987 , -0.915 , 1.571 , 0})
attach(bottle, gripper)
setQ({1.58598 , -1.77193 , -1.57229 , -1.44181 , 1.58623 , -0.0298101})
setQ({0.939804 , -1.75004 , -1.42469 , -1.2069 , 1.29026 , 0.187337})
setQ({0.206962 , -1.72521 , -1.2573 , -0.940472 , 0.954593 , 0.43361})
setQ({-0.525881 , -1.70038 , -1.0899 , -0.674048 , 0.618925 , 0.679883})
setQ({-0.946397 , -1.86019 , -1.60548 , -0.891129 , 0.808282 , 0.171128})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

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
setQ({2.53847 , -1.74792 , -1.57786 , -1.22811 , 1.7258 , 0.777721})
setQ({2.33394 , -1.75069 , -1.60085 , -1.21182 , 1.71722 , 0.734643})
setQ({1.46195 , -1.76252 , -1.69889 , -1.14236 , 1.68067 , 0.550982})
setQ({0.58997 , -1.77435 , -1.79693 , -1.07291 , 1.64411 , 0.367321})
setQ({-0.282015 , -1.78617 , -1.89496 , -1.00345 , 1.60756 , 0.183661})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

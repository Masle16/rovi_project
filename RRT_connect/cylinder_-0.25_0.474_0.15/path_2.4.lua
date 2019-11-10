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
setQ({2.70681 , -1.77082 , -0.65028 , -1.01509 , -0.575193 , -0.337759})
setQ({2.20864 , -1.65913 , -0.699803 , -0.806729 , 0.0207243 , -0.225329})
setQ({0.752604 , -1.33267 , -0.844548 , -0.197747 , 1.76245 , 0.103276})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

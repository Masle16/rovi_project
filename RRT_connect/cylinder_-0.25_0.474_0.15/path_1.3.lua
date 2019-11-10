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
setQ({1.96923 , -1.00282 , -1.4316 , -1.3983 , 1.49993 , -0.0983545})
setQ({1.44628 , -1.12105 , -1.19781 , -1.49527 , 1.54742 , -0.160595})
setQ({0.309498 , -1.37806 , -0.689619 , -1.70607 , 1.65066 , -0.295894})
setQ({-0.143088 , -1.376 , -1.73301 , -1.51086 , 1.40944 , 0.252016})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

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
setQ({2.17897 , -1.4734 , -0.277838 , -2.18062 , 0.811176 , -0.343411})
setQ({2.30146 , -1.40922 , 0.173372 , -2.49534 , 0.619662 , -0.429967})
setQ({0.903262 , -2.08073 , 0.541164 , -2.67054 , -1.05068 , 0.200991})
setQ({0.0975051 , -2.027 , -0.894048 , -1.307 , -0.0757966 , -0.287276})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

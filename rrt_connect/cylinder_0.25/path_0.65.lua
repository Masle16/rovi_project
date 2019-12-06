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
setQ({1.80488 , -1.59113 , -1.70572 , -1.20643 , 1.4347 , 0.407579})
setQ({1.39076 , -1.62009 , -1.74593 , -1.1683 , 1.45378 , 0.350535})
setQ({0.754568 , -1.66456 , -1.8077 , -1.10972 , 1.48308 , 0.262901})
setQ({0.118379 , -1.70904 , -1.86947 , -1.05115 , 1.51239 , 0.175267})
setQ({-0.517811 , -1.75352 , -1.93123 , -0.992574 , 1.54169 , 0.0876337})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

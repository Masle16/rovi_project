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
setQ({2.55723 , -1.91726 , -0.378414 , -1.46584 , 0.263017 , -0.980909})
setQ({1.85188 , -1.79975 , -1.05542 , -0.930145 , 0.862767 , -0.592251})
setQ({0.737706 , -1.61413 , -2.12484 , -0.0839433 , 1.81015 , 0.0216827})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

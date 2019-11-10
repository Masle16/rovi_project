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
setQ({2.62642 , -1.38125 , -0.553156 , -0.643979 , 0.950575 , 0.878763})
setQ({1.8943 , -1.46196 , -0.831999 , -0.700145 , 1.07073 , 0.70858})
setQ({0.370148 , -1.62998 , -1.4125 , -0.817072 , 1.32086 , 0.35429})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

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
setQ({1.67419 , -1.48109 , -0.713698 , -3.06807 , 1.67785 , -0.772929})
setQ({1.72527 , -1.37541 , -1.00212 , -2.50018 , 1.25486 , -0.779698})
setQ({1.89694 , -1.02022 , -1.97151 , -0.591493 , -0.166779 , -0.802449})
setQ({1.35077 , -1.33068 , -0.419004 , -1.04404 , -0.586823 , 1.08801})
setQ({-0.0927196 , -2.71275 , -0.310547 , -0.673621 , 1.02141 , 1.26349})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

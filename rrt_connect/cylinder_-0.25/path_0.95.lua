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
setQ({2.57527 , -1.80193 , -1.72127 , -1.53445 , 1.05063 , 0.475396})
setQ({2.21838 , -1.76808 , -1.6919 , -1.49564 , 1.13283 , 0.418148})
setQ({1.3152 , -1.68241 , -1.61756 , -1.39742 , 1.34084 , 0.27327})
setQ({0.412024 , -1.59675 , -1.54322 , -1.29921 , 1.54886 , 0.128393})
setQ({-0.491153 , -1.51108 , -1.46888 , -1.20099 , 1.75688 , -0.016485})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

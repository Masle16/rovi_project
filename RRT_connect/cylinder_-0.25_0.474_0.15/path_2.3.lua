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
setQ({2.18547 , -1.4265 , -0.280232 , -2.09468 , 0.239977 , -0.160395})
setQ({2.01829 , -1.39967 , -0.276955 , -1.93938 , -0.0298447 , -0.203101})
setQ({0.941083 , -1.22679 , -0.255838 , -0.938779 , -1.76834 , -0.478262})
setQ({-0.128967 , -1.7455 , -0.93051 , -0.3983 , -0.00284832 , -0.586253})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

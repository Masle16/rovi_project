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

setQ({2.185 , -1.795 , -1.987 , -0.915 , 1.571 , 0})
attach(bottle, gripper)
setQ({2.43107 , -1.77359 , -1.71185 , -1.73686 , 1.11529 , -0.436076})
setQ({2.17119 , -1.73662 , -1.73616 , -1.6392 , 1.0044 , -0.31348})
setQ({1.29464 , -1.61191 , -1.81815 , -1.30981 , 0.630367 , 0.100027})
setQ({0.418091 , -1.48719 , -1.90015 , -0.980427 , 0.256337 , 0.513535})
setQ({-0.45846 , -1.36248 , -1.98214 , -0.65104 , -0.117693 , 0.927042})
setQ({-0.957166 , -1.36373 , -1.91057 , -0.788402 , 0.657792 , 0.347453})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

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
setQ({2.41679 , -1.67014 , -0.472083 , -0.821979 , 1.08041 , 0.918393})
setQ({1.71906 , -1.37937 , -0.411072 , -0.905276 , 0.616467 , 1.07939})
setQ({0.488629 , -0.866607 , -0.303482 , -1.05217 , -0.201674 , 1.3633})
setQ({-1.03665 , -0.907408 , -0.47006 , -1.28076 , -0.500571 , 1.61334})
setQ({-2.24803 , -1.23615 , -0.490038 , -1.90505 , -0.122628 , 0.941408})
setQ({-1.87348 , -1.36642 , -1.70538 , -1.25946 , 0.58203 , 0.830285})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

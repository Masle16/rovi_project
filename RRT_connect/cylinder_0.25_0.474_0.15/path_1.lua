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
setQ({1.19082 , -1.43751 , -1.62893 , -1.31236 , 1.62449 , -0.568433})
setQ({1.03055 , -1.46866 , -1.59686 , -1.26064 , 1.59093 , -0.545896})
setQ({0.134531 , -1.64279 , -1.41757 , -0.971474 , 1.40331 , -0.419899})
setQ({-0.761484 , -1.81691 , -1.23828 , -0.682309 , 1.2157 , -0.293903})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

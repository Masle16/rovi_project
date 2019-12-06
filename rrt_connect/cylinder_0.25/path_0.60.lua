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
setQ({1.56727 , -1.46933 , -1.87304 , -0.809325 , 1.35269 , 0.420193})
setQ({1.13321 , -1.41599 , -1.84578 , -0.675306 , 1.32613 , 0.42001})
setQ({0.565792 , -1.34626 , -1.81015 , -0.50011 , 1.29142 , 0.419771})
setQ({-0.00163121 , -1.27652 , -1.77452 , -0.324913 , 1.2567 , 0.419532})
setQ({-0.509436 , -1.53777 , -1.70641 , -0.421672 , 1.11648 , 0.434345})
setQ({-0.919139 , -1.62623 , -1.99309 , -0.697276 , 1.24535 , 0.336529})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

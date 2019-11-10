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
setQ({2.43915 , -0.522215 , -1.0552 , -1.40198 , 0.438407 , 0.547658})
setQ({1.35738 , -0.863487 , -1.25608 , -1.74057 , -0.0912393 , -0.1899})
setQ({-0.223538 , -1.36223 , -1.54965 , -2.2354 , -0.86527 , -1.26777})
setQ({-1.99714 , -1.25548 , -1.56509 , -1.94612 , 0.389299 , -1.10945})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

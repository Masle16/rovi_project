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
setQ({1.83363 , -1.53772 , -0.956321 , -1.58133 , 1.64349 , -0.348272})
setQ({1.46689 , -1.60555 , -0.982502 , -1.59989 , 1.43094 , -0.198681})
setQ({0.380483 , -1.80647 , -1.06006 , -1.65486 , 0.801305 , 0.244459})
setQ({-0.705923 , -2.0074 , -1.13762 , -1.70984 , 0.17167 , 0.6876})
setQ({-1.70728 , -2.04602 , -1.39708 , -1.0348 , 0.704821 , 0.582512})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

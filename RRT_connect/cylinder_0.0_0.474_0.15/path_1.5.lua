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
setQ({2.37979 , -1.66385 , -0.676553 , -1.35529 , 1.18288 , -0.364814})
setQ({1.62024 , -1.64985 , -0.68486 , -1.32413 , 1.28675 , -0.141167})
setQ({0.194929 , -1.62358 , -0.700448 , -1.26566 , 1.48168 , 0.278512})
setQ({-1.23039 , -1.59731 , -0.716037 , -1.20718 , 1.67661 , 0.698191})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

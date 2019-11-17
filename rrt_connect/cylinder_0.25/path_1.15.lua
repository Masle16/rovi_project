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
setQ({1.2734 , -1.59392 , -1.35041 , -1.15886 , 1.17538 , -0.636827})
setQ({0.853809 , -1.45984 , -0.632822 , -1.38572 , 0.779762 , -1.27365})
setQ({0.556063 , -1.3647 , -0.123618 , -1.5467 , 0.49903 , -1.72555})
setQ({0.576632 , -1.87346 , -0.542174 , -1.56797 , 1.06219 , -0.970269})
setQ({-0.29989 , -2.18158 , -0.234111 , -1.29814 , 1.01957 , -0.431991})
setQ({-0.926637 , -2.19328 , -1.03126 , -1.30587 , 1.51889 , -0.220501})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

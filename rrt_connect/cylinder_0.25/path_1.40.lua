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
setQ({1.11195 , -1.57282 , -1.0165 , -1.406 , 1.06898 , 0.126292})
setQ({0.530908 , -1.41764 , 0.0349998 , -1.88 , 0.566958 , 0.252584})
setQ({0.379792 , -1.37728 , 0.30847 , -2.00328 , 0.436395 , 0.28543})
setQ({-0.575171 , -1.51208 , -0.270269 , -1.27134 , 0.712966 , -0.00215836})
setQ({-1.32063 , -1.86482 , -1.34626 , -1.20413 , 0.373285 , 0.0447878})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

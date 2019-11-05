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
setQ({1.90151 , -1.11947 , -1.77811 , -0.76417 , 1.81296 , -0.644567})
setQ({1.87338 , -1.12724 , -1.77996 , -0.76768 , 1.81379 , -0.628778})
setQ({1.03187 , -1.35967 , -1.83549 , -0.872689 , 1.83871 , -0.156438})
setQ({0.190361 , -1.5921 , -1.89101 , -0.977699 , 1.86363 , 0.315902})
setQ({-0.65115 , -1.82453 , -1.94654 , -1.08271 , 1.88855 , 0.788242})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

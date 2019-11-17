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
setQ({2.35992 , -1.14544 , -1.37429 , -0.372002 , 1.74393 , 0.8985})
setQ({1.79573 , -1.23232 , -1.18512 , -0.596734 , 1.62576 , 0.645751})
setQ({0.667334 , -1.40607 , -0.80678 , -1.04619 , 1.38941 , 0.140255})
setQ({-0.461056 , -1.57983 , -0.428442 , -1.49566 , 1.15305 , -0.365241})
setQ({-1.42082 , -1.81363 , -1.07394 , -1.42953 , 0.710573 , 0.240861})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

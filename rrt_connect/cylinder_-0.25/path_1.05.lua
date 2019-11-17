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
setQ({2.65291 , -1.70177 , -1.7521 , -1.62881 , 1.12516 , 0.538931})
setQ({2.50239 , -1.7356 , -1.09563 , -2.20822 , 0.59654 , 0.719425})
setQ({1.64722 , -1.89217 , -1.01273 , -1.91901 , 1.02637 , 0.535879})
setQ({0.777034 , -2.05149 , -0.92838 , -1.62473 , 1.46374 , 0.349111})
setQ({0.874526 , -1.74672 , -1.86464 , -1.30423 , 1.33343 , 0.287171})
setQ({-0.139737 , -1.77236 , -1.92882 , -1.11912 , 1.45222 , 0.143586})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

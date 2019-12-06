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
setQ({1.89901 , -1.26529 , -0.971729 , -1.81029 , 1.4389 , 0.689425})
setQ({1.88474 , -1.36698 , -0.890265 , -1.68806 , 1.37677 , 0.627753})
setQ({1.76663 , -2.2082 , -0.216365 , -0.676959 , 0.86282 , 0.11758})
setQ({0.636734 , -2.13998 , -0.50961 , -0.523034 , 1.22753 , -0.977167})
setQ({-0.211531 , -1.82806 , -0.805166 , -0.537185 , 1.21436 , 0.371137})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

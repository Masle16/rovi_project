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
setQ({2.30921 , -1.94251 , -1.17228 , -1.53195 , 1.42826 , -0.107726})
setQ({2.08931 , -1.95132 , -1.20019 , -1.47186 , 1.44618 , -0.0803408})
setQ({1.37889 , -1.97977 , -1.29038 , -1.27775 , 1.50408 , 0.00812961})
setQ({0.668474 , -2.00823 , -1.38057 , -1.08363 , 1.56198 , 0.0966001})
setQ({-0.0419463 , -2.03668 , -1.47075 , -0.889519 , 1.61988 , 0.185071})
setQ({-0.752366 , -2.06513 , -1.56094 , -0.695403 , 1.67778 , 0.273541})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

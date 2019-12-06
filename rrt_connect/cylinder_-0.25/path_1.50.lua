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
setQ({2.3731 , -2.18184 , -0.498551 , -0.214912 , 1.92016 , 0.552257})
setQ({2.37093 , -2.18176 , -0.49876 , -0.21535 , 1.91984 , 0.552238})
setQ({0.922937 , -2.12501 , -0.637547 , -0.507335 , 1.70675 , 0.53927})
setQ({-0.525056 , -2.06826 , -0.776334 , -0.79932 , 1.49366 , 0.526303})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

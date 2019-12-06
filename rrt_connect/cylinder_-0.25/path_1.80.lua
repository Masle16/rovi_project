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
setQ({2.36737 , -1.34103 , -0.403442 , -1.46859 , 0.775995 , 0.610506})
setQ({1.97386 , -1.3921 , -0.581076 , -1.40885 , 0.864837 , 0.542282})
setQ({0.409929 , -1.59505 , -1.28704 , -1.17143 , 1.21792 , 0.271141})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

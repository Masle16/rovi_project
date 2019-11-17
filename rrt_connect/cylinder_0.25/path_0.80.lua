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
setQ({1.55182 , -1.17704 , -1.57289 , -1.03259 , 1.52544 , -0.243265})
setQ({1.01231 , -1.22241 , -1.48468 , -1.011 , 1.49878 , -0.419731})
setQ({0.264628 , -1.28529 , -1.36244 , -0.981079 , 1.46185 , -0.664289})
setQ({-0.483059 , -1.34816 , -1.24019 , -0.951161 , 1.42491 , -0.908846})
setQ({-0.855857 , -1.64315 , -1.56156 , -0.647698 , 1.65524 , -0.501918})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

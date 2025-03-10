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
setQ({2.50171 , -2.00655 , -1.3801 , -0.961704 , 1.63181 , 0.253118})
setQ({2.29756 , -1.9939 , -1.40275 , -0.971724 , 1.5991 , 0.227408})
setQ({1.95762 , -1.97283 , -1.44046 , -0.988408 , 1.54464 , 0.184596})
setQ({1.61769 , -1.95177 , -1.47817 , -1.00509 , 1.49019 , 0.141783})
setQ({1.27775 , -1.9307 , -1.51588 , -1.02178 , 1.43573 , 0.0989714})
setQ({0.937811 , -1.90963 , -1.55359 , -1.03846 , 1.38127 , 0.0561593})
setQ({0.597874 , -1.88857 , -1.59129 , -1.05515 , 1.32681 , 0.0133472})
setQ({0.257937 , -1.8675 , -1.629 , -1.07183 , 1.27235 , -0.0294649})
setQ({-0.0820005 , -1.84643 , -1.66671 , -1.08851 , 1.21789 , -0.0722769})
setQ({-0.421938 , -1.82537 , -1.70442 , -1.1052 , 1.16344 , -0.115089})
setQ({-0.761875 , -1.8043 , -1.74213 , -1.12188 , 1.10898 , -0.157901})
setQ({-0.974608 , -1.84935 , -1.82987 , -0.872174 , 1.08162 , -0.224333})
setQ({-1.11464 , -1.7421 , -2.04256 , -0.90201 , 1.24429 , -0.0872215})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

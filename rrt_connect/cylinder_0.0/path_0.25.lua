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
setQ({1.93564 , -1.79596 , -1.98607 , -0.907636 , 1.55673 , 0.00762581})
setQ({1.68627 , -1.79692 , -1.98514 , -0.900272 , 1.54246 , 0.0152516})
setQ({1.65404 , -1.65625 , -2.02328 , -0.831783 , 1.5483 , 0.203646})
setQ({1.69936 , -1.55338 , -1.8207 , -0.809168 , 1.59991 , 0.27881})
setQ({1.54294 , -1.58691 , -1.87077 , -0.820205 , 1.58793 , 0.249149})
setQ({1.31427 , -1.63593 , -1.94396 , -0.83634 , 1.57041 , 0.205785})
setQ({1.08559 , -1.68495 , -2.01715 , -0.852475 , 1.5529 , 0.162421})
setQ({0.83675 , -1.69751 , -2.01447 , -0.861534 , 1.55491 , 0.144374})
setQ({0.587906 , -1.71007 , -2.01179 , -0.870592 , 1.55692 , 0.126328})
setQ({0.339063 , -1.72263 , -2.0091 , -0.87965 , 1.55893 , 0.108281})
setQ({0.0902188 , -1.73519 , -2.00642 , -0.888709 , 1.56094 , 0.090234})
setQ({-0.158625 , -1.74775 , -2.00373 , -0.897767 , 1.56295 , 0.0721872})
setQ({-0.407469 , -1.76032 , -2.00105 , -0.906825 , 1.56497 , 0.0541404})
setQ({-0.656312 , -1.77288 , -1.99837 , -0.915883 , 1.56698 , 0.0360936})
setQ({-0.905156 , -1.78544 , -1.99568 , -0.924942 , 1.56899 , 0.0180468})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})

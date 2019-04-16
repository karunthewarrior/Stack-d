import kinematics as kin

def straight(s,d):
	_,x_list,_ = kin.inverse_kinematics(d,init=s)
	return x_list


if __name__=="__main__":
	p1 = [0.3,0.05,0.2]
	p2 = [0.3,0.05,0.1]
	print(straight(p1,p2))
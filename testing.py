import numpy as np

# Implement Dynamics for Accelerations Here #
M_q = np.zeros((3,3))        
M_q = ([Jx,0,-Jx*stheta],[0,m1*l1^2+m2*l2^2+Jy*cphi^2+Jz*sphi^2,(Jy-Jz)*sphi*cphi*ctheta],[-Jx*stheta,(Jy-Jz)*sphi*cphi*ctheta,(m1*l1^2+m2*l2^2+Jy*sphi^2+Jz*cphi^2)*ctheta^2+Jx*stheta^2])
c_q_qd = np.zeros((3,1))
c_q_qd = ([-thetad^2*(Jz-Jy)*sphi*cphi+psid^2*(Jz-Jy)*sphi*cphi*ctheta^2-thetad*psid*ctheta*(Jx-(Jz-Jy)(cphi^2-sphi^2))],[psid^2*stheta*ctheta*(-Jx+m1*l1^2+m2*l2^2+Jy*sphi^2+Jz*cphi^2)-2*phid*thetad*(Jz-Jy)*sphi*cphi-phid*psid*ctheta*(-Jx+(Jz-Jy)(cphi^2-sphi^2))],[thetad^2*(Jz-Jy)*sphi*cphi*stheta-phid*thetad*ctheta*(Jx+(Jz-Jy)(cphi^2-sphi^2))-2*phid*psid*(Jz-Jy)*ctheta^2*sphi*cphi+2*thetad*psid*stheta*ctheta*(Jx-m1*l1^2-m2*l2^2-Jy*sphi^2-Jz*cphi^2)])
dP_dq = [0,(m1*l1-m2*l2)*g*ctheta,0]
Q = [d*(fl-fr), l1*(fl+fr)*cphi, l1*(fl+fr)*ctheta*sphi+d*(fr-fl)*stheta]

xdot = [0]*6
matrixdd = np.zeros((3,1))
#matrixdd = np.linalg.solve()
xdot[3] = matrixdd[0] #phidd
xdot[4] = matrixdd[1] #thetadd
xdot[5] = matrixdd[2] #psidd

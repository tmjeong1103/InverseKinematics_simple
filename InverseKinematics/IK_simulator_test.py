from dataclasses import dataclass
import numpy as np
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)

# Setup Robot
@dataclass
class LinkNode:
    """Class for link node info"""
    name: str
    id: int
    mother: int
    child: np.ndarray

    q: int          # joint angle
    a: np.ndarray   # joint axis vector (relative to parent)
    b: np.ndarray   # joint relative position (relative to parent)
    p: np.ndarray   # position in world coordinates
    R: np.ndarray   # attitude in relative coordinates

@dataclass
class TargetNode:
    name: str
    p: np.ndarray  # position in world coordinates
    R: np.ndarray  # attitude in world coordinates

class RobotObject:

    def __init__(self):

        self.ulink = {
            1: LinkNode(name='OP_Middle_Hip', id=1, child=np.array([11, 17]), mother=0
                        , q=0, a=np.zeros(3), b=np.zeros(3), p=np.zeros(3), R=np.eye(3)),

            11: LinkNode(name='OP_L_Hip_1', id=11, child=np.array([12]), mother=1
                         , q=0, a=np.zeros(3), b=np.zeros(3), p=np.zeros(3), R=np.eye(3)),
            12: LinkNode(name='OP_L_Hip_2', id=12, child=np.array([13]), mother=11
                         , q=0, a=np.zeros(3), b=np.zeros(3), p=np.zeros(3), R=np.eye(3)),
            13: LinkNode(name='OP_L_Hip_3', id=13, child=np.array([14]), mother=12
                         , q=0, a=np.zeros(3), b=np.zeros(3), p=np.zeros(3), R=np.eye(3)),
            14: LinkNode(name='OP_L_Knee', id=14, child=np.array([15]), mother=13
                         , q=0, a=np.zeros(3), b=np.zeros(3), p=np.zeros(3), R=np.eye(3)),
            15: LinkNode(name='OP_L_Ankle_1', id=15, child=np.array([16]), mother=14
                         , q=0, a=np.zeros(3), b=np.zeros(3), p=np.zeros(3), R=np.eye(3)),
            16: LinkNode(name='OP_L_Ankle_2', id=16, child=np.array([0]), mother=15
                         , q=0, a=np.zeros(3), b=np.zeros(3), p=np.zeros(3), R=np.eye(3)),

            17: LinkNode(name='OP_R_Hip_1', id=17, child=np.array([18]), mother=1
                         , q=0, a=np.zeros(3), b=np.zeros(3), p=np.zeros(3), R=np.eye(3)),
            18: LinkNode(name='OP_R_Hip_2', id=18, child=np.array([19]), mother=17
                         , q=0, a=np.zeros(3), b=np.zeros(3), p=np.zeros(3), R=np.eye(3)),
            19: LinkNode(name='OP_R_Hip_3', id=19, child=np.array([20]), mother=18
                         , q=0, a=np.zeros(3), b=np.zeros(3), p=np.zeros(3), R=np.eye(3)),
            20: LinkNode(name='OP_R_Knee', id=20, child=np.array([21]), mother=19
                         , q=0, a=np.zeros(3), b=np.zeros(3), p=np.zeros(3), R=np.eye(3)),
            21: LinkNode(name='OP_R_Ankle_1', id=21, child=np.array([22]), mother=20
                         , q=0, a=np.zeros(3), b=np.zeros(3), p=np.zeros(3), R=np.eye(3)),
            22: LinkNode(name='OP_R_Ankle_2', id=22, child=np.array([0]), mother=21
                         , q=0, a=np.zeros(3), b=np.zeros(3), p=np.zeros(3), R=np.eye(3))
        }
        self.Target = {
            1: TargetNode(name='OP_Middle_Hip', p=np.zeros(3), R=np.eye(3)),

            11: TargetNode(name='OP_L_Hip', p=np.zeros(3),R=np.eye(3)),
            12: TargetNode(name='OP_L_Hip2', p=np.zeros(3),R=np.eye(3)),
            13: TargetNode(name='OP_L_Hip3', p=np.zeros(3),R=np.eye(3)),
            14: TargetNode(name='OP_L_Knee', p=np.zeros(3), R=np.eye(3)),
            15: TargetNode(name='OP_L_Ankle1', p=np.zeros(3), R=np.eye(3)),
            16: TargetNode(name='OP_L_Ankle2', p=np.zeros(3), R=np.eye(3)),                          #End point

            17: TargetNode(name='OP_R_Hip', p=np.zeros(3),R=np.eye(3)),
            18: TargetNode(name='OP_R_Hip2', p=np.zeros(3),R=np.eye(3)),
            19: TargetNode(name='OP_R_Hip3', p=np.zeros(3),R=np.eye(3)),
            20: TargetNode(name='OP_R_Knee', p=np.zeros(3), R=np.eye(3)),
            21: TargetNode(name='OP_R_Ankle1', p=np.zeros(3), R=np.eye(3)),
            22: TargetNode(name='OP_R_Ankle2', p=np.zeros(3), R=np.eye(3))}                        #End point

    def print_link_name(self, link_id):

        if link_id not in self.ulink.keys():
            return KeyError("Does not have matching link node id.")

        querying_node = self.ulink[link_id]

        print("ID: ", querying_node.id)
        print("NAME: ", querying_node.name)
        print("Child: ", querying_node.child)

    def forward_kinematics(self, link_id):
        if link_id == 0:
            return

        if link_id != 1:
            mom = self.ulink[link_id].mother
            self.ulink[link_id].p = np.dot(self.ulink[mom].R, self.ulink[link_id].b) + self.ulink[mom].p
            self.ulink[link_id].R = np.dot(self.ulink[mom].R, self.Rodrigues(self.ulink[link_id].a, self.ulink[link_id].q))

        if self.ulink[link_id].child[0] != 0:
            for i in self.ulink[link_id].child:
                self.forward_kinematics(i)
        else:
            return

    def Rodrigues(self,a,q):
        # a : unit vector, q : rotation angle
        E = np.eye(3)
        a_norm = np.linalg.norm(a, ord=2)                               #Euclidean Distance, L2-norm
        if a_norm < 10**-6:                                             #a_norm is zeros
            R = np.eye(3)
        else:
            a = a / a_norm                                              #unit vector
            th = a_norm*q
            a_hat = np.array([[0,-a[2],a[1]],                           #skew symmetric matrix, kajita robotics : p34
                              [a[2],0,-a[0]],
                              [-a[1],a[0],0]])
            R = E + a_hat*np.sin(th) + a_hat.dot(a_hat)*(1-np.cos(th))    #Rodrigues` formula = E + a_hat*sinq + a_hat^2*(1-cosq), kajita robotics : p35
        return R

    def CalJacobian(self, idx):
        target = self.ulink[idx[-1]].p                                  #absolute target position
        J = np.zeros((6, idx.size))       # 6 X q_size
        n = 0
        for i in idx:
            mom = self.ulink[i].mother
            a = np.dot(self.ulink[mom].R, self.ulink[i].a)               #joint axis in world frame
            J[:, n] = np.append(np.cross(a, target-self.ulink[i].p), a)  # kajita robotics p59 (2.74)
            n = n + 1
        return J

    def FindRoute(self, to):
        idx = []
        mom = self.ulink[to].mother
        if mom == 0:
            return print("Link node ","'",to,"'"," is Base node.")
        if mom == 1:
            idx = np.append(to, idx)                                         #idx is not included Base. (Link node 1)
        else:
            idx = np.append(to,idx)
            idx = np.append(self.FindRoute(mom),idx)
        return idx

    def CalcVWerr(self, idx):
        if idx in self.ulink:
            p_err = self.Target[idx].p - self.ulink[idx].p
            r_err = np.transpose(self.ulink[idx].R).dot(self.Target[idx].R)
            w_err = self.ulink[idx].R.dot(self.rot2omega(r_err))
            err = np.append(p_err, w_err)
            return err
        else:
            return print(idx, "index is not End point.. so I don`t calculate")

    def rot2omega(self, r):
        el = np.array([r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1]])
        norm_el = np.linalg.norm(el, ord=2)
        if norm_el >= 10**-6:
            w = np.arctan2(norm_el, np.trace(r)-1)/norm_el*el
        elif r[0,0] > 0 and r[1,1] > 0 and r[2,2] > 0:
            w = np.array([0,0,0])
        else:
            w = np.pi/2*[r[0,0]+1, r[1,1]+1, r[2,2]+1]
        return w

    def InverseKinematics(self, target_idx):
        lam = 0.5                                                               # tuning value
        idx = self.FindRoute(target_idx)
        self.forward_kinematics(1)
        err = self.CalcVWerr(target_idx)
        for i in range(0, 10):
            if np.linalg.norm(err, ord=2) <= 10**-6:
                break
            J = self.CalJacobian(idx)
            dq = lam*(np.linalg.solve(J, err))
            n = 0
            for j in idx:
                self.ulink[j].q = self.ulink[j].q + dq[n]
                n = n+1
            self.forward_kinematics(1)
            err = self.CalcVWerr(target_idx)

    #input the degree
    def rpy2rot(self,r,p,y):
        c_ph = np.cos(np.deg2rad(r))
        c_th = np.cos(np.deg2rad(p))
        c_ps = np.cos(np.deg2rad(y))

        s_ph = np.sin(np.deg2rad(r))
        s_th = np.sin(np.deg2rad(p))
        s_ps = np.sin(np.deg2rad(y))

        rpy = np.array([[c_ps*c_th, -s_ps*c_ph+c_ps*s_th*s_ph, s_ps*s_ph+c_ps*s_th*c_ph],
                        [s_ps*c_th, c_ps*c_ph+s_ps*s_th*s_ph, -c_ps*s_ph+s_ps*s_th*c_ph],
                        [-s_th, c_th*s_ph, c_th*c_ph]])
        return rpy

    def DrawAllJoints(self,fig,j):
        if j != 0:
            i = self.ulink[j].mother
            if i != 0:
                self.connect3D(fig, self.ulink[i].p, self.ulink[j].p)
            if self.ulink[j].child[0] != 0:
                for i in self.ulink[j].child:
                    self.DrawAllJoints(fig,i)

    def connect3D(self,fig,p1,p2):
        xs = [p1[0], p2[0]]
        ys = [p1[1], p2[1]]
        zs = [p1[2], p2[2]]

        out = Arrow3D(xs, ys, zs, mutation_scale=20, arrowstyle='-', color='k',
                      linestyle='-', linewidth=1)
        fig.add_artist(out)

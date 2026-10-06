import numpy as np, trimesh
D='mycobot_description/urdf/mycobot_280_arduino/'
def verts(name):
    sc=trimesh.load(D+name+'.dae',force='scene')
    return sc.dump(concatenate=True).vertices*0.001
def rz(a):
    c,s=np.cos(a),np.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])
vis={'left1':(0.039,-0.0133,-0.012),'right1':(-0.039,-0.0133,-0.012),'base':(0,0,-0.012)}
B=verts('gripper_base')+np.array(vis['base'])
print('base y %.4f..%.4f x %.4f..%.4f z %.4f..%.4f'%(B[:,1].min(),B[:,1].max(),B[:,0].min(),B[:,0].max(),B[:,2].min(),B[:,2].max()))
for g in (-0.7,-0.3,0.15):
    for side,j3,j1,sgn in (('left',(-0.012,-0.0025),(-0.027,0.016),1),('right',(0.012,-0.0025),(0.027,0.016),-1)):
        p1=np.array([*j3,0])+rz(sgn*g)@np.array([*j1,0])
        W=verts('gripper_'+side+'1')+np.array(vis[side+'1'])+p1
        top=W[W[:,1]>W[:,1].max()-0.004]
        print(g,side,'tip y %.4f'%W[:,1].max(),' x at tip %.4f..%.4f'%(top[:,0].min(),top[:,0].max()),' z %.4f..%.4f'%(W[:,2].min(),W[:,2].max()))

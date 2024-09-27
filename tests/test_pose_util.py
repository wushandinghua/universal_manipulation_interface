# %%
import sys
import os

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

# %%
import numpy as np
from diffusion_policy.model.common.rotation_transformer import RotationTransformer
from umi.common.pose_util import rot6d_to_mat, mat_to_rot6d, adapt4fr


# %%
def test():
    poseFr = [606.122, -101.697, 91.379, -120.1, 0.005, -89.966]
    pose0 = adapt4fr(poseFr, toFr=False)
    print(pose0)
    pose0 = [ 1.01018480e-01,  3.41946253e-01,  7.42907405e-02,  1.61163946e-03,-1.24403810e-03, -3.14108487e+00]
    pose1 = adapt4fr(pose0)
    print(pose1)
    N = 100
    d6 = np.random.normal(size=(N,6))
    rt = RotationTransformer(from_rep='rotation_6d', to_rep='matrix')
    gt_mat = rt.forward(d6)
    mat = rot6d_to_mat(d6)
    assert np.allclose(gt_mat, mat)
    
    to_d6 = mat_to_rot6d(mat)
    to_d6_gt = rt.inverse(mat)
    assert np.allclose(to_d6, to_d6_gt)
    
    gt_mat = rt.forward(d6[0])
    mat = rot6d_to_mat(d6[0])
    assert np.allclose(gt_mat, mat)
    
    to_d6 = mat_to_rot6d(mat)
    to_d6_gt = rt.inverse(mat)
    assert np.allclose(to_d6, to_d6_gt)
    
    
if __name__ == "__main__":
    test()

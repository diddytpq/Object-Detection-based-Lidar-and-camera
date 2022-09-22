import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
# from ops import Voxelization, nms_cuda

from spconv.utils import Point2VoxelCPU3d
from spconv.pytorch.utils import PointToVoxel, gather_features_by_pc_voxel_id



class VoxelLayer(nn.Module):
    def __init__(self, voxel_size, point_cloud_range, max_num_points, max_voxels, device, pillar_flag = False):
        super().__init__()

        self.voxel_gen_layer = PointToVoxel(vsize_xyz = voxel_size, #[0.1, 0.1, 0.1],
                                coors_range_xyz= point_cloud_range,   #[-80, -80, -6, 80, 80, 6],
                                num_point_features=4,
                                max_num_voxels = max_voxels,  #5000,
                                max_num_points_per_voxel = max_num_points,  #5
                                device = device)

        self.pillar_flag = pillar_flag

    @torch.no_grad()
    def forward(self, batched_pts):
        '''
        batched_pts: list[tensor], len(batched_pts) = bs
        return: 
               voxels: (p1 + p2 + ... + pb, num_points, c), 
               coors_batch: (p1 + p2 + ... + pb, 1 + 3), 
               npoints_per_voxel: (p1 + p2 + ... + pb, ), (b: batch size)
        '''
        voxels, coors, npoints_per_voxel = [], [], []
        if self.pillar_flag:
            pillar_coors = []
        for i, pts in enumerate(batched_pts):
            # input_pts = pts[:,:3]
            voxels_out, coors_out, num_points_per_voxel_out = self.voxel_gen_layer(pts, empty_mean=False)
            coors_out = coors_out.flip(-1) # (z, y, x) -> (x, y, z)

            if self.pillar_flag:
                pillar_coors_out = coors_out
                pillar_coors_out[-1] = 0
                pillar_coors.append(pillar_coors_out.long())

            voxels.append(voxels_out)
            coors.append(coors_out.long())
            npoints_per_voxel.append(num_points_per_voxel_out)
        
        
        voxels = torch.cat(voxels, dim = 0)
        npoints_per_voxel = torch.cat(npoints_per_voxel, dim=0) # (p1 + p2 + ... + pb, )
        coors_batch = []

        for i, cur_coors in enumerate(coors):
            coors_batch.append(F.pad(cur_coors, (1, 0), value=i))
        coors_batch = torch.cat(coors_batch, dim=0) # (p1 + p2 + ... + pb, 1 + 3)

        if self.pillar_flag:
            pillar_coors_batch = []

            for i, pillar_coor in enumerate(pillar_coors):
                pillar_coors_batch.append(F.pad(pillar_coor, (1, 0), value=i))

            pillar_coors_batch = torch.cat(pillar_coors_batch, dim=0) # (p1 + p2 + ... + pb, 1 + 3)

            return voxels, coors_batch, npoints_per_voxel, pillar_coors_batch

        return voxels, coors_batch, npoints_per_voxel




# if __name__ == "__main__":

#     np.random.seed(2222)

#     device = torch.device("cuda:0")

#     test_layer = PillarLayer(voxel_size=[0.1, 0.1, 0.1],
#                             point_cloud_range=[-80, -80, -6, 80, 80, 6],
#                             max_voxels=5000,
#                             max_num_points=32,
#                             device = device)

#     pc = np.random.uniform(-2, 8, size=[1000, 4]).astype(np.float32)

#     pc_tensor = torch.from_numpy(pc).to(device)
#     voxels_th, indices_th, num_p_in_vx_th = test_layer([pc_tensor])

#     print(voxels_th.size())
#     print(indices_th.size())
#     print(num_p_in_vx_th.size())

#     voxels_th = voxels_th.cpu().numpy()
#     indices_th = indices_th.cpu().numpy()
#     num_p_in_vx_th = num_p_in_vx_th.cpu().numpy()

#     print(voxels_th[120])
#     print(indices_th[120])
#     print(num_p_in_vx_th[120])


import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
# from ops import Voxelization, nms_cuda

from ops import Voxelization, nms_cuda



class VoxelLayer(nn.Module):
    def __init__(self, voxel_size, point_cloud_range, max_num_points, max_voxels):
        super().__init__()
        self.voxel_layer = Voxelization(voxel_size=voxel_size,
                                        point_cloud_range=point_cloud_range,
                                        max_num_points=max_num_points,
                                        max_voxels=max_voxels)

    @torch.no_grad()
    def forward(self, batched_pts):
        '''
        batched_pts: list[tensor], len(batched_pts) = bs
        return: 
               pillars: (p1 + p2 + ... + pb, num_points, c), (n번째 voxel, n번째 voxel의 i번째 포인트, {x,y,z,r})
               coors_batch: (p1 + p2 + ... + pb, 1 + 3),   (n번째 voxel, {batch_id,x,y,z})
               num_points_per_pillar: (p1 + p2 + ... + pb, ), (b: batch size) (n번째 voxel, point 개수)
        '''
        pillars, coors, npoints_per_pillar = [], [], []
        for i, pts in enumerate(batched_pts):
            voxels_out, coors_out, num_points_per_voxel_out = self.voxel_layer(pts) 
            # voxels_out: (max_voxel, num_points, c), coors_out: (max_voxel, 3)
            # num_points_per_voxel_out: (max_voxel, )
            pillars.append(voxels_out)
            coors.append(coors_out.long())
            npoints_per_pillar.append(num_points_per_voxel_out)
        
        pillars = torch.cat(pillars, dim=0) # (p1 + p2 + ... + pb, num_points, c)
        npoints_per_pillar = torch.cat(npoints_per_pillar, dim=0) # (p1 + p2 + ... + pb, )
        coors_batch = []
        for i, cur_coors in enumerate(coors):
            coors_batch.append(F.pad(cur_coors, (1, 0), value=i))
        coors_batch = torch.cat(coors_batch, dim=0) # (p1 + p2 + ... + pb, 1 + 3)

        return pillars, coors_batch, npoints_per_pillar # (16000, 32, 4), (16000, 4), (16000)

class PillarEncoder(nn.Module):
    def __init__(self, voxel_size, point_cloud_range, in_channel, out_channel): # in_channel= 9, out_channel = 64
        super().__init__()
        self.out_channel = out_channel
        self.vx, self.vy = voxel_size[0], voxel_size[1]
        self.x_offset = voxel_size[0] / 2 + point_cloud_range[0]
        self.y_offset = voxel_size[1] / 2 + point_cloud_range[1]
        self.x_l = int((point_cloud_range[3] - point_cloud_range[0]) / voxel_size[0]) #voxel의 최대 x 크기
        self.y_l = int((point_cloud_range[4] - point_cloud_range[1]) / voxel_size[1]) #voxel의 최대 y 크기

        self.conv = nn.Conv1d(in_channel, out_channel, 1, bias=False)
        self.bn = nn.BatchNorm1d(out_channel, eps=1e-3, momentum=0.01)

    def forward(self, pillars, coors_batch, npoints_per_pillar):
        '''
        pillars: (p1 + p2 + ... + pb, num_points, c), c = 4
        coors_batch: (p1 + p2 + ... + pb, 1 + 3)
        npoints_per_pillar: (p1 + p2 + ... + pb, )
        return:  (bs, out_channel, y_l, x_l)
        '''

        device = pillars.device
        # 1. calculate offset to the points center (in each pillar)
        offset_pt_center = pillars[:, :, :3] - torch.sum(pillars[:, :, :3], dim=1, keepdim=True) / npoints_per_pillar[:, None, None] # (p1 + p2 + ... + pb, num_points, 3)
        
        # 2. calculate offset to the pillar center
        x_offset_pi_center = pillars[:, :, :1] - (coors_batch[:, None, 1:2] * self.vx + self.x_offset) # (p1 + p2 + ... + pb, num_points, 1)
        y_offset_pi_center = pillars[:, :, 1:2] - (coors_batch[:, None, 2:3] * self.vy + self.y_offset) # (p1 + p2 + ... + pb, num_points, 1)

        # 3. encoder          x,y,z,r      x,y,z              x,                      y     
        features = torch.cat([pillars, offset_pt_center, x_offset_pi_center, y_offset_pi_center], dim=-1) # (p1 + p2 + ... + pb, num_points, 9)
        features[:, :, 0:1] = x_offset_pi_center # tmp
        features[:, :, 1:2] = y_offset_pi_center # tmp   

        # 4. find mask for (0, 0, 0) and update the encoded features
        # a very beautiful implementation
        voxel_ids = torch.arange(0, pillars.size(1)).to(device) # (num_points, )
        mask = voxel_ids[:, None] < npoints_per_pillar[None, :] # (num_points, p1 + p2 + ... + pb)
        mask = mask.permute(1, 0).contiguous()  # (p1 + p2 + ... + pb, num_points)
        features *= mask[:, :, None] #([16000, 32, 9])

        # 5. embedding
        features = features.permute(0, 2, 1).contiguous() # (p1 + p2 + ... + pb, 9, num_points) ([16000, 9, 32])
        features = F.relu(self.bn(self.conv(features)))  # (p1 + p2 + ... + pb, out_channels, num_points) ([16000, 64, 32])
        pooling_features = torch.max(features, dim=-1)[0] # (p1 + p2 + ... + pb, out_channels) ([16000, 64])

        # 6. pillar scatter
        batched_canvas = []
        bs = coors_batch[-1, 0] + 1  #데이터의 최대 batch size 
        for i in range(bs):
            cur_coors_idx = coors_batch[:, 0] == i
            cur_coors = coors_batch[cur_coors_idx, :]
            cur_features = pooling_features[cur_coors_idx]

            canvas = torch.zeros((self.x_l, self.y_l, self.out_channel), dtype=torch.float32, device=device)
            canvas[cur_coors[:, 1], cur_coors[:, 2]] = cur_features #canvas[x좌표, y좌표] = cur_features ([432, 496, 64])
            canvas = canvas.permute(2, 1, 0).contiguous() #([64, 432, 496])
            batched_canvas.append(canvas)
        batched_canvas = torch.stack(batched_canvas, dim=0) # (bs, in_channel, self.y_l, self.x_l)

        return batched_canvas


class Backbone(nn.Module):
    def __init__(self, in_channel, out_channels, layer_nums, layer_strides=[2, 2, 2]): #(in_channel=64, 
        super().__init__()                                                             #out_channels=[64, 128, 256], layer_nums=[3, 5, 5])
        assert len(out_channels) == len(layer_nums)
        assert len(out_channels) == len(layer_strides)

        self.multi_blocks = nn.ModuleList()
        for i in range(len(layer_strides)):
            blocks = []
            blocks.append(nn.Conv2d(in_channel, out_channels[i], 3, stride=layer_strides[i], bias=False, padding=1))
            blocks.append(nn.BatchNorm2d(out_channels[i], eps=1e-3, momentum=0.01))
            blocks.append(nn.ReLU(inplace=True)) # inplace 하면 input으로 들어온 것 자체를 수정하겠다는 뜻. 메모리 usage가 좀 좋아짐. 

            for _ in range(layer_nums[i]):
                blocks.append(nn.Conv2d(out_channels[i], out_channels[i], 3, bias=False, padding=1))
                blocks.append(nn.BatchNorm2d(out_channels[i], eps=1e-3, momentum=0.01))
                blocks.append(nn.ReLU(inplace=True)) 
 
            in_channel = out_channels[i]
            self.multi_blocks.append(nn.Sequential(*blocks))

        # in consitent with mmdet3d
        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')

    def forward(self, x):
        '''
        x: (b, c, y_l, x_l). Default: (6, 64, 496, 432)
        return: list[]. Default: [(6, 64, 248, 216), (6, 128, 124, 108), (6, 256, 62, 54)]
        '''
        outs = []
        for i in range(len(self.multi_blocks)):
            x = self.multi_blocks[i](x)
            outs.append(x)
        return outs

class Head(nn.Module):
    def __init__(self, in_channel, n_anchors, n_classes): #in_channel=384, n_anchors=2*nclasses(3), n_classes=nclasses)
        super().__init__()
        
        self.conv_cls = nn.Conv2d(in_channel, n_anchors*n_classes, 1)
        self.conv_reg = nn.Conv2d(in_channel, n_anchors*7, 1)
        self.conv_dir_cls = nn.Conv2d(in_channel, n_anchors*2, 1)

        # in consitent with mmdet3d
        conv_layer_id = 0
        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.normal_(m.weight, mean=0, std=0.01)
                if conv_layer_id == 0:
                    prior_prob = 0.01
                    bias_init = float(-np.log((1 - prior_prob) / prior_prob))
                    nn.init.constant_(m.bias, bias_init)
                else:
                    nn.init.constant_(m.bias, 0)
                conv_layer_id += 1

    def forward(self, x):
        '''
        x: (bs, 384, 248, 216)
        return: 
              bbox_cls_pred: (bs, n_anchors*3, 248, 216) 
              bbox_pred: (bs, n_anchors*7, 248, 216)
              bbox_dir_cls_pred: (bs, n_anchors*2, 248, 216)
        '''
        bbox_cls_pred = self.conv_cls(x)
        bbox_pred = self.conv_reg(x)
        bbox_dir_cls_pred = self.conv_dir_cls(x)
        
        return bbox_cls_pred, bbox_pred, bbox_dir_cls_pred


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


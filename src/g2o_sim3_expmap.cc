//
// Created by pang on 19-7-3.
//

/**
* This file is part of MultiCol-SLAM
*
* Copyright (C) 2015-2016 Steffen Urban <urbste at googlemail.com>
* For more information see <https://github.com/urbste/MultiCol-SLAM>
*
* MultiCol-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* MultiCol-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with MultiCol-SLAM . If not, see <http://www.gnu.org/licenses/>.
*/
#include "g2o_sim3_expmap.h"

namespace ORB_SLAM2
{

    // 顶点 用于优化sim3 //添加了 keypoint_to_cam
    VertexSim3Expmap_Multi::VertexSim3Expmap_Multi(
            std::unordered_map<size_t, int>& kp_to_cam1,
            std::unordered_map<size_t, int>& kp_to_cam2)
            : g2o::BaseVertex<7, g2o::Sim3>(),
              keypoint_to_cam1(kp_to_cam1),
              keypoint_to_cam2(kp_to_cam2)
    {
        _marginalized = false;
        _fix_scale = false;
    }

}
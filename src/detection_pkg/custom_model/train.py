import copy
import json
import os
from pathlib import Path
import pickle
import shutil
import time
import re 
import fire
import numpy as np
import torch
from google.protobuf import text_format

import second.data.kitti_common as kitti
import torchplus
from second.builder import target_assigner_builder, voxel_builder
from second.core import box_np_ops
from second.data.preprocess import merge_second_batch, merge_second_batch_multigpu
from second.protos import pipeline_pb2
from second.pytorch.builder import (box_coder_builder, input_reader_builder,
                                    lr_scheduler_builder, optimizer_builder,
                                    second_builder)
from second.utils.log_tool import SimpleModelLog
from second.utils.progress_bar import ProgressBar
import psutil

from utils.train_utils import build_network, _worker_init_fn, example_convert_to_torch

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

def train(config_path,
          model_dir = "/train_model/ext",
          display_step=50,
          summary_step=5,
          pretrained_path=None,
          pretrained_include=None,
          pretrained_exclude=None,
          freeze_include=None,
          freeze_exclude=None,
          multi_gpu=False,
          measure_time=False,
          resume=False):

    model_dir = str(Path(model_dir).resolve())
    model_dir = Path(model_dir)
    model_dir.mkdir(parents=True, exist_ok=True)

    result_path = model_dir / 'results'
    config_file_bkp = "pipeline.config"

    if isinstance(config_path, str):
    # directly provide a config object. this usually used
    # when you want to train with several different parameters in
    # one script.
    # 모델의 config 파일 불러오기
        config = pipeline_pb2.TrainEvalPipelineConfig()
        with open(config_path, "r") as f:
            proto_str = f.read()
            text_format.Merge(proto_str, config)

    with (model_dir / config_file_bkp).open("w") as f:
        f.write(proto_str)

    input_cfg = config.train_input_reader
    eval_input_cfg = config.eval_input_reader
    model_cfg = config.model.second
    train_cfg = config.train_config

    # config file대로 신경망 제작
    net = build_network(model_cfg, measure_time).to(device)

    target_assigner = net.target_assigner
    voxel_generator = net.voxel_generator

    net_parallel = net

    optimizer_cfg = train_cfg.optimizer      # adam optimizer
    loss_scale = train_cfg.loss_scale_factor # -1

    fastai_optimizer = optimizer_builder.build(optimizer_cfg,
                                                net,
                                                mixed=False,
                                                loss_scale=loss_scale)

    if loss_scale < 0:
        loss_scale = "dynamic"

    amp_optimizer = fastai_optimizer

    lr_scheduler = lr_scheduler_builder.build(optimizer_cfg, amp_optimizer, train_cfg.steps)

    float_dtype = torch.float32

    collate_fn = merge_second_batch
    num_gpu = 1

    ######################
    # PREPARE DataLoader
    ######################

    dataset = input_reader_builder.build(
        input_cfg,
        model_cfg,
        training=True,
        voxel_generator=voxel_generator,
        target_assigner=target_assigner,
        multi_gpu=multi_gpu)

    eval_dataset = input_reader_builder.build(
        eval_input_cfg,
        model_cfg,
        training=False,
        voxel_generator=voxel_generator,
        target_assigner=target_assigner)

    dataloader = torch.utils.data.DataLoader(dataset,
                                            batch_size=input_cfg.batch_size * num_gpu,
                                            shuffle=True,
                                            num_workers=input_cfg.preprocess.num_workers * num_gpu,
                                            pin_memory=False,
                                            collate_fn=collate_fn,
                                            worker_init_fn=_worker_init_fn,
                                            drop_last=not multi_gpu)

    eval_dataloader = torch.utils.data.DataLoader(eval_dataset,
                                                batch_size=eval_input_cfg.batch_size, # only support multi-gpu train
                                                shuffle=False,
                                                num_workers=eval_input_cfg.preprocess.num_workers,
                                                pin_memory=False,
                                                collate_fn=merge_second_batch)

    ######################
    # TRAINING
    ######################
    t = time.time()

    start_step = net.get_global_step()
    total_step = train_cfg.steps

    steps_per_eval = train_cfg.steps_per_eval
    clear_metrics_every_epoch = train_cfg.clear_metrics_every_epoch

    amp_optimizer.zero_grad()
    step_times = []
    step = start_step

    try:
        while True:
            if clear_metrics_every_epoch:
                net.clear_metrics()

            for example in dataloader:
                lr_scheduler.step(net.get_global_step())
                time_metrics = example["metrics"]
                example.pop("metrics")
                example_torch = example_convert_to_torch(example, float_dtype)

                batch_size = example["anchors"].shape[0]

                ret_dict = net_parallel(example_torch)
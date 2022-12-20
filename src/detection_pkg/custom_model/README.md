# Cutsom model based pcdet

## install
    https://github.com/open-mmlab/OpenPCDet/blob/master/docs/INSTALL.md

## Create model
    
- NuScenes Dataset
    ```
    OpenPCDet
    ├── data
    │   ├── nuscenes
    │   │   │── v1.0-trainval (or v1.0-mini if you use mini)
    │   │   │   │── samples
    │   │   │   │── sweeps
    │   │   │   │── maps
    │   │   │   │── v1.0-trainval  
    ├── pcdet
    ├── tools
    ```
    ```
    pip install nuscenes-devkit==1.0.5
    python -m pcdet.datasets.nuscenes.nuscenes_dataset --func create_nuscenes_infos \
                                                       --cfg_file tools/cfgs/dataset_configs/nuscenes{v1.0-mini, v1.0-trainval, v1.0test}_dataset.yaml \
                                                       --version v1.0-{v1.0-mini, v1.0-trainval, v1.0test}
    ```

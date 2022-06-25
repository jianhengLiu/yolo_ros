# 1. NVIDIA GeForce RTX 3060 Laptop GPU with CUDA capability sm_86 is not compatible with the current PyTorch installation.   
https://blog.csdn.net/tp_0moyi0/article/details/120751325

**reinstall pytorch with corresponding CUDA version**

# 2. Error processing request: 'Upsample' object has no attribute 'recompute_scale_factor'
https://blog.csdn.net/t80824724/article/details/123548016?spm=1001.2101.3001.6661.1&utm_medium=distribute.pc_relevant_t0.none-task-blog-2%7Edefault%7ECTRLIST%7Edefault-1-123548016-blog-123582773.pc_relevant_antiscanv4&depth_1-utm_source=distribute.pc_relevant_t0.none-task-blog-2%7Edefault%7ECTRLIST%7Edefault-1-123548016-blog-123582773.pc_relevant_antiscanv4&utm_relevant_index=1

**reinstall pytorch with version under 1.9.0**

uninstall pytorch
```
conda uninstall torch
conda uninstall torchvision
conda uninstall cudatoolkit

pip uninstall torch
pip uninstall torchvision
pip uninstall torchaudio
```

install pytorch-1.9.0
```
conda install pytorch==1.9.0 torchvision==0.10.0 torchaudio==0.9.0 cudatoolkit=11.3 -c pytorch -c conda-forge
```

or 

modified `upsample.py` as follows:

https://github.com/ultralytics/yolov5/issues/6948#issuecomment-1075548218
import torch

if torch.cuda.is_available():
    print("CUDA is available")
    print("cuDNN version:", torch.backends.cudnn.version())
else:
    print("CUDA is NOT available")

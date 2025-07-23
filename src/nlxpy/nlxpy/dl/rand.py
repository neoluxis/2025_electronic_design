import torch
import os
import random
import numpy as np

randseed = 0

def set_random_seed(seed=0, use_deterministic=False):
    """Set random seed for reproducibility."""
    global randseed
    randseed = seed
    print(f"Set random seed: {seed}")
    torch.manual_seed(seed)
    torch.random.manual_seed(seed)
    random.seed(seed)
    np.random.seed(seed)
    if use_deterministic:
        print(f"Use deterministic algorithms...")
        torch.use_deterministic_algorithms(True)
        os.environ['CUBLAS_WORKSPACE_CONFIG'] = ':4096:8'
        os.environ['CUBLAS_WORKSPACE_CONFIG'] = ':16:8'
        
def get_random_seed():
    """Get the current random seed."""
    return randseed

if __name__ == "__main__":
    # Example usage
    set_random_seed(42, use_deterministic=True)
    print(f"Current random seed: {get_random_seed()}")
    # Test random number generation
    print(f"Random number: {torch.rand(1)}")
    print(f"Random integer: {torch.randint(0, 100, (1,))}")
    print(f"Random choice: {random.choice([1, 2, 3, 4, 5])}")
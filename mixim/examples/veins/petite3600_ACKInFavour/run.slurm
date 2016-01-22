#!/bin/bash
#SBATCH --job-name=1hAckInFavour
#SBATCH --output=slurm.out
#SBATCH --error=slurm.err
#SBATCH --nodes=1
  
srun -l vanet_sim.sh

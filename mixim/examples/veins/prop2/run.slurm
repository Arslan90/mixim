#!/bin/bash
#SBATCH --job-name=2hAck
#SBATCH --output=slurm.out
#SBATCH --error=slurm.err
#SBATCH --nodes=2
  
srun -l vanet_sim.sh

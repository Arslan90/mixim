#!/bin/bash
#SBATCH --job-name=convergeCast2H2
#SBATCH --output=slurm.out
#SBATCH --error=slurm.err
#SBATCH --nodes=2
  
srun -l vanet_sim.sh
